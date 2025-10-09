// Copyright (c) 2025, ANTOBOT LTD.
// All rights reserved.


// Description: This file is to support the multi thread and manger the thread safety.

// Contacts: yu.chen@nicecart.ai

#ifndef FASTLIO2_THREADSAFETY_H
#define FASTLIO2_THREADSAFETY_H
/**
 * @brief 线程安全的队列模板类
 * 用于在不同线程之间安全地传递数据
 */
template<typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue() = default;

    /**
     * @brief 向队列中添加元素
     * @param item 要添加的元素
     */
    void push(T&& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(item));
        condition_.notify_one();
    }

    /**
     * @brief 从队列中取出元素（阻塞版本）
     * @return T 队列中的元素
     */
    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !queue_.empty() || shutdown_; });

        if (shutdown_ && queue_.empty()) {
            return T{};
        }

        T result = std::move(queue_.front());
        queue_.pop();
        return result;
    }

    /**
     * @brief 尝试从队列中取出元素（非阻塞版本）
     * @param item 输出参数，存储取出的元素
     * @return bool 是否成功取出元素
     */
    bool tryPop(T& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief 获取队列大小
     * @return size_t 队列中元素的数量
     */
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief 检查队列是否为空
     * @return bool 队列是否为空
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief 关闭队列，唤醒所有等待的线程
     */
    void shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        condition_.notify_all();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
    bool shutdown_ = false;
};
#endif //FASTLIO2_THREADSAFETY_H