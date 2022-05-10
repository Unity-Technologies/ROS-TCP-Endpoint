#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

// A threadsafe-queue.
template <class T>
class SafeQueue
{
    public:
    SafeQueue(void)
        : q()
        , m()
        , c()
    {}

    ~SafeQueue(void)
    {}

    // Add an element to the queue.
    void enqueue(T t)
    {
        std::lock_guard<std::mutex> lock(m);
        q.push(std::move(t));
        c.notify_one();
    }

    void interruptWaiters()
    {
        c.notify_one();
    }

    // Get the "front"-element.
    // If the queue is empty, wait till a element is avaiable.
    // This operation may fail if interrupted, but that doesn't necessarily mean
    // there's no more data.
    bool tryAwaitDequeue(T* result)
    {
        std::unique_lock<std::mutex> lock(m);
        if(q.empty())
        {
            // release lock as long as the wait and reaquire it afterwards.
            c.wait(lock);

            if(q.empty())
                return false;
        }
        *result = std::move(q.front());
        q.pop();
        return true;
    }

    // Get the front element, put it in result and return true.
    // If the queue is empty, return false.
    bool tryDequeue(T* result)
    {
        if(q.empty())
            return false;

        std::unique_lock<std::mutex> lock(m);

        if(q.empty())
            return false;

        *result = std::move(q.front());
        q.pop();
        return true;
    }

    private:
    std::queue<T> q;
    mutable std::mutex m;
    std::condition_variable c;
};