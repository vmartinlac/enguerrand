
#pragma once

#include <mutex>
#include <condition_variable>

class Semaphore
{
public:

    Semaphore(int count = 0) : mCount(count)
    {
    }

    void up()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mCount++;
        mCondition.notify_one();
    }

    void down()
    {
        std::unique_lock<std::mutex> lock(mMutex);

        while(mCount == 0)
        {
            mCondition.wait(lock);
        }

        mCount--;
    }

private:

    std::mutex mMutex;
    std::condition_variable mCondition;
    int mCount;
};

