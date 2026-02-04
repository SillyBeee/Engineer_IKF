#include "inferencer.hpp"
#include "thread_pool.hpp"


template <typename ModelType, typename InputType, typename OutputType>
class InferencerPool{
public:
    InferencerPool(std::string model_path , int thread_num);

    int Init();

    int Put(InputType& input);

    int Get(OutputType& output);

    ~InferencerPool();
protected:
    int GetInferencerIndex();
private:
    size_t thread_num_;
    std::string model_path_;

    long long id;
    std::mutex idMtx, queueMtx;
    std::unique_ptr<ThreadPool> thread_pool_;
    std::queue<std::future<OutputType>> results_queue_;
    std::vector<std::shared_ptr<Inferencer>> inferencers_;
};


template <typename ModelType, typename InputType, typename OutputType>
InferencerPool<ModelType, InputType, OutputType>::InferencerPool(std::string model_path , int thread_num)
    : model_path_(model_path), thread_num_(thread_num), id(0) {
    thread_pool_ = std::make_unique<ThreadPool>(thread_num_);
}

