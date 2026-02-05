#include "inferencer.hpp"
#include "thread_pool.hpp"
#include <mutex>
#include <new>
#include <condition_variable>
#include <queue>


template <typename ModelType, typename InputType, typename OutputType>
class InferencerPool{
public:
    InferencerPool(std::string model_path , int thread_num);

    int Init();

    int Put(InputType input);

    int Get(OutputType& output);

    std::vector<std::shared_ptr<ModelType>>& GetInferencers(){
        return inferencers_;
    }

    ~InferencerPool();
protected:
    int GetInferencerIndex();
private:
    std::string model_path_;
    size_t thread_num_;
    
    long long id;
    std::mutex idMtx, queueMtx;
    std::unique_ptr<ThreadPool> thread_pool_;
    std::queue<std::future<OutputType>> results_queue_;
    std::vector<std::shared_ptr<ModelType>> inferencers_;
    std::mutex inferencer_mtx_;
    std::condition_variable inferencer_cv_;
    std::queue<int> available_indices_;
    
    int AcquireAvailableInferencer();
    void ReleaseInferencer(int index);
};

//构造函数
template <typename ModelType, typename InputType, typename OutputType>
InferencerPool<ModelType, InputType, OutputType>::InferencerPool(std::string model_path , int thread_num)
    : model_path_(model_path), thread_num_(thread_num), id(0) {}

//初始化函数
template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::Init(){
    try{
        this->thread_pool_ = std::make_unique<ThreadPool>(thread_num_);
        for(size_t i = 0; i < thread_num_; ++i){
            auto inferencer = std::make_shared<ModelType>(model_path_);
            inferencers_.push_back(inferencer);
            available_indices_.push(static_cast<int>(i));
        }
    }
    catch(const std::bad_alloc& e){
        RCLCPP_ERROR(rclcpp::get_logger("InferencerPool"), "Failed to initialize InferencerPool: %s", e.what());
        return -1;
    }
    return 0;
}    

template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::GetInferencerIndex() {
    std::lock_guard<std::mutex> lock(idMtx);
    int  index = id % thread_num_;
    ++id;
    return index;
}

template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::Put(InputType input){
    {
        std::lock_guard<std::mutex> lock(queueMtx);
        if(results_queue_.size() >= thread_num_ * 4){
            RCLCPP_DEBUG(rclcpp::get_logger("InferencerPool"), "InferencerPool queue is full");
            return -1;
        }
    }
    auto future = thread_pool_->Submit([this, input = std::move(input)]() mutable {
        int inferencer_index = AcquireAvailableInferencer();
        auto inferencer = inferencers_[inferencer_index];
        OutputType result = inferencer->Infer(input);
        ReleaseInferencer(inferencer_index);
        return result;
    });
    {
        std::lock_guard<std::mutex> lock(queueMtx);
        results_queue_.push(std::move(future));
        RCLCPP_DEBUG(rclcpp::get_logger("InferencerPool"), "Put input into inferencer pool, current queue size: %zu", results_queue_.size());
    }
    return 0;
}
template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::Get(OutputType& output) {
    std::lock_guard<std::mutex> lock(queueMtx);
    if (results_queue_.empty()) {
        return -1;
    }
    output = results_queue_.front().get();
    results_queue_.pop();
    return 0;
}

template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::AcquireAvailableInferencer() {
    std::unique_lock<std::mutex> lock(inferencer_mtx_);
    inferencer_cv_.wait(lock, [this]() {
        return !available_indices_.empty();
    });
    int index = available_indices_.front();
    available_indices_.pop();
    return index;
}
template <typename ModelType, typename InputType, typename OutputType>
void InferencerPool<ModelType, InputType, OutputType>::ReleaseInferencer(int index) {
    {
        std::lock_guard<std::mutex> lock(inferencer_mtx_);
        available_indices_.push(index);
    }
    inferencer_cv_.notify_one();
}

template <typename ModelType, typename InputType, typename OutputType>
InferencerPool<ModelType, InputType, OutputType>::~InferencerPool(){
    while(!results_queue_.empty()){
        OutputType output = results_queue_.front().get();
        results_queue_.pop();
    }
    inferencers_.clear();
    thread_pool_.reset();
}