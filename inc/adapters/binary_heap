//
// Created by Peter on 9/21/2025.
//

#ifndef HEAP_H
#define HEAP_H
#include <vector>
#include <concepts>
#include "base_heap_.h"

template <
    typename T,
    HeapableContainer Container = std::vector<T>,
    HeapFunctor<Container> Compare = std::less<T>>
requires std::same_as<T, typename Container::value_type> && std::is_object_v<T>
class BinaryHeap final : public BaseHeap<T, Container, Compare>{
public:
    using BaseHeap_ = BaseHeap<T, Container, Compare>;
    using value_type = typename Container::value_type;
    using size_type = typename Container::size_type;

    using const_reference = const value_type&;
    using const_iterator = typename Container::const_iterator;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
protected:

    void heapify_up(size_t from) noexcept(std::is_nothrow_move_assignable_v<value_type>) override {
        value_type val = std::move(this->data_[from]);
        while (from) {
            size_t par = (from - 1) >> 1;
            if (!this->compare_(this->data_[par], val)) break;
            this->data_[from] = std::move(this->data_[par]);
            from = par;
        }
        this->data_[from] = std::move(val);
    }
    void heapify_down(size_t idx) noexcept(std::is_nothrow_move_assignable_v<value_type>) override {
        value_type val = std::move(this->data_[idx]);
        const size_t n = this->data_.size();

        while (true) {
            size_t child_idx = 2 * idx + 1;
            if (child_idx >= n) break;

            size_t best = child_idx;
            if (child_idx + 1 < n && this->compare_(this->data_[child_idx],\
                    this->data_[child_idx + 1])) {
                ++best;
            }
            if (!this->compare_(val, this->data_[best])) break;

            this->data_[idx] = std::move(this->data_[best]);
            idx = best;
        }
        this->data_[idx] = std::move(val);
    }


    void build_heap() noexcept {
        if (this->size() <= 1) return;
        for (size_t i = this->size() / 2; i-- > 0; ) {
            this->heapify_down(i);
        }
    }

public:
    //Delegate to the constructors of BaseHeap
    template<typename ...Arg>
    explicit BinaryHeap(Arg&&... args)
    : BaseHeap_(std::forward<Arg>(args)...) {
        this->build_heap();
    }

    BinaryHeap(const BinaryHeap&) = default;
    BinaryHeap(BinaryHeap&&) = default;
    BinaryHeap& operator=(const BinaryHeap&) = default;
    BinaryHeap& operator=(BinaryHeap&&) = default;

    ~BinaryHeap() = default;

    template <std::input_iterator InputIt>
    requires std::constructible_from<value_type, std::iter_reference_t<InputIt>>
    void push_range(InputIt first, InputIt last) {
        if constexpr (std::random_access_iterator<InputIt>) {
            const size_type n = this->size();
            const auto m = static_cast<size_type>(last - first);

            if constexpr (requires (size_type i) { this->data_.reserve(i); }) {
                this->data_.reserve(n + m);
            }

            auto log2n = [](const size_type x) -> size_type {
                if (x < 2) return 1;
                return static_cast<size_type>(std::bit_width(x) - 1);
            };

            //rebuild if m >= (n / max(1, log2(n))), O(n + m)
            if (!n || m >= n / log2n(n)) {
                for (; first != last; ++first) {
                    this->data_.emplace_back(std::forward<decltype(*first)>(*first));
                }
                build_heap();
                return;
            }
            //otherwise heapify one-by-one, O(m log(n + m))
            for (; first != last; ++first) {
                this->data_.emplace_back(std::forward<decltype(*first)>(*first));
                heapify_up(this->size() - 1U);
            }
            return;
        }
        //for input-iterator
        for (; first != last; ++first) {
            this->data_.emplace_back(std::forward<decltype(*first)>(*first));
            heapify_up(this->size() - 1U);
        }
    }

    template <std::ranges::input_range Rng>
    requires std::constructible_from<value_type, std::ranges::range_reference_t<Rng>>
    void push_range(Rng&& rng) {
        push_range(std::ranges::begin(rng), std::ranges::end(rng));
    }

};
#endif //HEAP_H
