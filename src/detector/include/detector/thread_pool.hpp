#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <cassert>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

//线程池实现

class ThreadPool{
public: 
    ThreadPool():
    ThreadPool(std::thread::hardware_concurrency()){}

    ThreadPool(size_t max_thread_num):
        quit_(false),
        current_thread_num_(0),
        idle_thread_num_(0),
        max_threads_num_(max_thread_num){}
    
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    template <typename Func, typename... Ts>
    auto Submit(Func&& func , Ts&&... params)->std::future<typename std::result_of<Func(Ts...)>::type>{
        using ReturnType = typename std::result_of<Func(Ts...)>::type;
        using PackagedTask = std::packaged_task<ReturnType()>;

        //封装任务
        auto execute = std::bind(std::forward<Func>(func), std::forward<Ts>(params)...);
        auto task = std::make_shared<PackagedTask>(
           std::move(execute));
        auto result  = task->get_future();
        
        std::lock_guard<std::mutex> lock(mutex_);
        assert(!quit_);

        //加入任务队列
        tasks_.emplace([task]()
                           { (*task)(); });
        //如果有空闲线程则通知    
        if(idle_thread_num_ > 0){
            cv_.notify_one();
        }
        //若线程池未满则创建新线程
        else if (current_thread_num_ < max_threads_num_)
            {
                std::thread t(&ThreadPool::worker, this);
                assert(threads_.find(t.get_id()) == threads_.end());
                threads_[t.get_id()] = std::move(t);
                ++current_thread_num_;
        }
        //异步返回结果
        return result;    
    }

    size_t GetThreadNum() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return current_thread_num_;
    }




private:
    void worker(){
        while(true){
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> uniqueLock(mutex_);
                ++idle_thread_num_;
                auto hasTimedout = !cv_.wait_for(uniqueLock,
                                                    std::chrono::seconds(WAIT_SECONDS),
                                                    [this]()
                                                    {
                                                        return quit_ || !tasks_.empty();
                                                    });
                --idle_thread_num_;
                if (tasks_.empty())
                {
                    if (quit_)
                    {
                        --current_thread_num_;
                        return;
                    }
                    if (hasTimedout)
                    {
                        --current_thread_num_;
                        JoinFinishedThreads();
                        finishedThreadIDs_.emplace(std::this_thread::get_id());
                        return;
                    }
                }
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
            
        }
    }

    void JoinFinishedThreads()
        {
            while (!finishedThreadIDs_.empty())
            {
                auto id = std::move(finishedThreadIDs_.front());
                finishedThreadIDs_.pop();
                auto iter = threads_.find(id);

                assert(iter != threads_.end());
                assert(iter->second.joinable());

                iter->second.join();
                threads_.erase(iter);
            }
        }

    static constexpr size_t WAIT_SECONDS = 2;    


    bool quit_;
    size_t current_thread_num_;
    size_t idle_thread_num_;
    size_t max_threads_num_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::function<void()>> tasks_;
    std::queue<std::thread::id> finishedThreadIDs_;
    std::unordered_map<std::thread::id, std::thread> threads_;
};


#endif // THREAD_POOL_HPP