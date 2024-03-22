Week 6 - PREWORK
=========================

Before we start developing code that uses threading, we will have a look at two implementations of threading that allow us to understand basic concepts.

Simple_Threading
-----------------------------------------

Threads are created on two functions `incrementNum` and `printNum`.  We pass `num` and `numMutex` to both of these functions, so they can share data and a `mutex` which synchronises access so that both functions are not writing/reading the data simultaneously (which causes corruption of data). 

Attempt following

* Do not secure `num` by a mutex, what occurs?
* Remove the scope `{` and `}` that surrounds locking the mutex, is there a change?
* Can we the lock and unlock  [mutex](https://en.cppreference.com/w/cpp/thread/mutex) , is there a change?
* Can we use a [scoped lock](https://en.cppreference.com/w/cpp/thread/scoped_lock) or [lock guard](https://en.cppreference.com/w/cpp/thread/lock_guard) instead?
* What happens if we do not have the infinite while loop in the threaded functions?  Make incrementNum function finish looping if the number reaches 1000 for instance).
