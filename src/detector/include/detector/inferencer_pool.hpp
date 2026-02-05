#include "inferencer.hpp"
#include "thread_pool.hpp"
#include <mutex>
#include <new>


template <typename ModelType, typename InputType, typename OutputType>
class InferencerPool{
public:
    InferencerPool(std::string model_path , int thread_num);

    int Init();

    int Put(InputType& input);

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
int InferencerPool<ModelType, InputType, OutputType>::Put(InputType& input){
    std::lock_guard<std::mutex> lock(queueMtx);
    if(results_queue_.size() >= thread_num_ * 4){
        RCLCPP_DEBUG(rclcpp::get_logger("InferencerPool"), "InferencerPool queue is full");
        return -1;
    }
    results_queue_.push(thread_pool_->Submit(&ModelType::Infer, inferencers_[GetInferencerIndex()], std::forward<InputType>(input)));
    RCLCPP_DEBUG(rclcpp::get_logger("InferencerPool"), "Put input into inferencer pool, current queue size: %zu", results_queue_.size());
    return 0;
}


template <typename ModelType, typename InputType, typename OutputType>
int InferencerPool<ModelType, InputType, OutputType>::Get(OutputType& output){
    std::lock_guard<std::mutex> lock(queueMtx);
    if(results_queue_.empty()){
        return -1;
    }
    output = results_queue_.front().get();
    results_queue_.pop();
    return 0;
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