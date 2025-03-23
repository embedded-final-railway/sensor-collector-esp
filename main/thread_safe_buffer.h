#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <iostream>

template <typename T>
class ThreadSafeBuffer {
private:
    std::vector<T> buffer;
    size_t head, tail, count;
    std::mutex mtx;
    std::condition_variable not_full, not_empty;

public:
    ThreadSafeBuffer(size_t size);
    void insert(const T& item);
    T extract();
    void batch_insert(const std::vector<T>& items);
    std::vector<T> batch_extract(size_t num_items);
    
    bool is_empty();
    bool is_full();
    size_t size() const;
    size_t current_count() const;
};
