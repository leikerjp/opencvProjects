#pragma once
#include <queue>
#include <mutex>
#include <iostream>

/*
 * class MultiThreadQueue
 *
 * The MultiThreadQueue class is STL based queue that is thread safe.
 *
 */
template <typename T>
class MultiThreadQueue
{
	/********** Private Members **********/
	std::mutex mLock;
	std::queue<T> qBuf;
	unsigned int qSize;

public:
	/********** Public Members **********/

	/*
	 * Delete the default constructor (force users to provide a queue size.
	 */
	MultiThreadQueue() = delete;

	/*
	 * MultiThreadQueue(void) :
	 *
	 * Description:
	 * Constructor with custom queue size.
	 *
	 * Inputs:
	 *		unsigned int inSize			size of queue
	 *
	 * Outputs:
	 *		N/A
	 */
	MultiThreadQueue(const unsigned int inSize) :
		qSize(inSize)
	{}

	/*
	 * push(T& value) :
	 *
	 * Description:
	 * This function pushes data to the back of the queue (if there is space).
	 *
	 * Inputs:
	 *		T& value				value written to back of queue
	 *
	 * Outputs:
	 *		bool (return val)		indicator of read success. false indciates no value was read
	 */
	bool push(T& value)
	{
		mLock.lock();
		bool success = false;
		if (qBuf.size() <= qSize)
		{
			qBuf.push(value);
			success = true;
		}
		mLock.unlock();
		return success;
	}

	/*
	 * pop(T& value) :
	 *
	 * Description:
	 * This function grabs the data at the front of the queue (if there is some available).
	 *
	 * Inputs:
	 *		N/A
	 *
	 * Outputs:
	 *		T& value				value read from front of queue
	 *		bool (return val)		indicator of read success. false indciates no value was read
	 */
	bool pop(T& value)
	{
		mLock.lock();
		bool success = false;
		if (!qBuf.empty())
		{
			value = qBuf.front();
			qBuf.pop();
			success = true;
		}
		mLock.unlock();
		return success;
	}
};
