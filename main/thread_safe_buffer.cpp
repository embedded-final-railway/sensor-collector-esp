#include "thread_safe_buffer.h"

// Constructor
template <typename T>
ThreadSafeBuffer<T>::ThreadSafeBuffer(size_t size)
    : buffer(size), head(0), tail(0), count(0) {
}

// Insert a single item
template <typename T>
void ThreadSafeBuffer<T>::insert(const T &item) {
    std::unique_lock<std::mutex> lock(mtx);
    not_full.wait(lock, [this]() { return count < buffer.size(); });

    buffer[head] = item;
    head = (head + 1) % buffer.size();
    count++;

    not_empty.notify_one();
}

// Extract a single item
template <typename T>
T ThreadSafeBuffer<T>::extract() {
    std::unique_lock<std::mutex> lock(mtx);
    not_empty.wait(lock, [this]() { return count > 0; });

    T item = buffer[tail];
    tail = (tail + 1) % buffer.size();
    count--;

    not_full.notify_one();
    return item;
}

// Batch insert
template <typename T>
void ThreadSafeBuffer<T>::batch_insert(const std::vector<T> &items) {
    std::unique_lock<std::mutex> lock(mtx);
    not_full.wait(lock, [this, &items]() { return count + items.size() <= buffer.size(); });

    for (const auto &item : items) {
        buffer[head] = item;
        head = (head + 1) % buffer.size();
        count++;
    }

    not_empty.notify_all();
}

// Batch extract
template <typename T>
std::vector<T> ThreadSafeBuffer<T>::batch_extract(size_t num_items) {
    std::unique_lock<std::mutex> lock(mtx);
    not_empty.wait(lock, [this, num_items]() { return count >= num_items; });

    std::vector<T> items;
    items.reserve(num_items);
    for (size_t i = 0; i < num_items; ++i) {
        items.push_back(buffer[tail]);
        tail = (tail + 1) % buffer.size();
        count--;
    }

    not_full.notify_all();
    return items;
}

// Check if the buffer is empty
template <typename T>
bool ThreadSafeBuffer<T>::is_empty() {
    std::lock_guard<std::mutex> lock(mtx);
    return count == 0;
}

// Check if the buffer is full
template <typename T>
bool ThreadSafeBuffer<T>::is_full() {
    std::lock_guard<std::mutex> lock(mtx);
    return count == buffer.size();
}

// Get the size of the buffer
template <typename T>
size_t ThreadSafeBuffer<T>::size() const {
    return buffer.size();
}

// Get the current count of items
template <typename T>
size_t ThreadSafeBuffer<T>::current_count() const {
    std::lock_guard<std::mutex> lock(mtx);
    return count;
}