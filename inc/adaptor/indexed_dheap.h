#ifndef INDEXED_DHEAP_H
#define INDEXED_DHEAP_H
#pragma once
#include <cstdint>
#include <stdexcept>
#include "base_heap_.h"
#include <unordered_map>
#include <Containers/inplace_vector.h>
#include "../compare.h"

namespace urlicht{

template <
    typename T,
    std::size_t d = 4,
    concepts::random_access_container Container = std::vector<T>,
    concepts::comparison_functor<T> Compare = compare::less<>,
    std::incrementable KeyType = uint64_t,
    concepts::contiguous_container KeyContainer = std::vector<KeyType>,
    concepts::unordered_map PositionMap = std::unordered_map<KeyType, typename Container::size_type>>
requires std::same_as<T, typename Container::value_type> &&
         std::is_object_v<T> && (d >= 2) &&
         std::same_as<typename KeyContainer::value_type, KeyType> &&
         std::same_as<typename PositionMap::key_type, KeyType> &&
         std::same_as<typename PositionMap::mapped_type, typename Container::size_type> &&
         std::default_initializable<KeyType>
class indexed_d_ary_heap final : public detail::base_heap<T, Container, Compare> {
public:
    // Common type alias
    using value_type = typename Container::value_type;
    using size_type = typename Container::size_type;
    using difference_type = std::ptrdiff_t;
    using container_type = Container;
    using value_compare = Compare;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using iterator = typename Container::const_iterator;
    using const_iterator = typename Container::const_iterator;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    // Specific type alias
    using key_type = KeyType;
    using key_storage = KeyContainer;
    using position_storage = PositionMap;

    static consteval size_type arity() noexcept { return d; }

private:
    key_storage keys_{}; //keys_[pos] stores the key of data_[pos]
    position_storage positions_{}; // positions_[key] -> the index of the value associated with the key in data_[pos]
    key_type next_key{};

    [[nodiscard]] static constexpr size_t parent(const size_t i) noexcept { return (i - 1) / d; }
    [[nodiscard]] static constexpr size_t left(const size_t i) noexcept { return d * i + 1; }

    constexpr void erase_at(typename position_storage::const_iterator it) {
        // the key must exist; UB otherwise
        const auto idx = it->second;
        this->positions_.erase(it);

        if (idx == this->size() - 1U) [[unlikely]] {
            this->data_.pop_back();
            this->keys_.pop_back();
            return;
        }

        this->data_[idx] = std::move(this->data_.back());
        this->data_.pop_back();
        this->keys_[idx] = keys_.back();
        this->keys_.pop_back();
        // since keys_[idx] exists
        this->positions_.find(this->keys_[idx])->second = idx;

        this->update_at(idx);
    }

    constexpr void update_at(const size_type idx) {
        if (idx != 0U && this->compare_(this->data_[parent(idx)], this->data_[idx])) {
            this->heapify_up(idx);
        } else {
            this->heapify_down(idx);
        }
    }

    // Strong exception gurantee
    template <typename ...Args>
    constexpr void insert_all_noheapify(const key_type key, const size_type size, Args&& ...args) {
        bool p_done = false, d_done = false;
        try {
            this->positions_.emplace(key, size); p_done = true;
            this->data_.emplace_back(std::forward<Args>(args)...); d_done = true;
            // keys_ is of the same size as data_, so we use emplace_back
            this->keys_.emplace_back(key);
        }
        catch(...){
            if (p_done) this->positions_.erase(key);
            if (d_done) this->data_.pop_back();
            throw;
        }
    }

    // No exception guarentee
    constexpr void heapify_up(size_type from) {
        value_type val = std::move(this->data_[from]);
        const key_type k = keys_[from];
        size_type par = parent(from);

        while(from && this->compare_(this->data_[par], val)) {
            this->data_[from] = std::move(this->data_[par]);
            this->keys_[from] = this->keys_[par];
            this->positions_.find(this->keys_[from])->second = from;
            from = par;
            par = parent(from);
        }

        this->data_[from] = std::move(val);
        this->keys_[from] = k;
        this->positions_.find(k)->second = from;
    }

    // No exception guarantee
    constexpr void heapify_down(size_type idx) {
        const size_type n = this->data_.size();
        value_type val = std::move(this->data_[idx]);
        key_type k = keys_[idx];

        while (true) {
            size_type ch = d * idx + 1;
            if (ch >= n) break;

            size_type best = ch;
            if constexpr (d == 2) {
                best += static_cast<size_type>(ch + 1 < n &&
                                               this->compare_(this->data_[ch], this->data_[ch + 1]));
            } else {
                const size_type last = std::min(d * idx + d, n - 1);
                for (++ch; ch <= last; ++ch) {
                    if (this->compare_(this->data_[best], this->data_[ch])) best = ch;
                }
            }
            this->data_[idx] = std::move(this->data_[best]);
            this->keys_[idx] = this->keys_[best];
            this->positions_.find(this->keys_[idx])->second = idx;
            idx = best;
        }
        this->data_[idx] = std::move(val);
        this->keys_[idx] = k;
        this->positions_.find(k)->second = idx;
    }

    constexpr void build_heap() {
        const auto m = this->data_.size();
        if (m <= 1) {
          return;
        }
        for(size_type i = (m - 2) / d + 1U; i-- > 0U;) {
            this->heapify_down(i);
        }
    }

    constexpr void initialize_keys() { // Initialize keys_ and positions_ after data_ is initialized
        if constexpr (concepts::reservable_container<position_storage>) {
            this->positions_.reserve(this->data_.size());
        } if constexpr (concepts::reservable_container<key_storage>) {
            this->keys_.reserve(this->data_.size());
        }
        for (size_type i = 0; i < this->size(); ++i) {
            bool p_done = false;
            const auto key = this->next_key;
            try {
                this->positions_.emplace(key, i); p_done = true;
                this->keys_.emplace_back(key);
            }
            catch(...){
                if (p_done) this->positions_.erase(key);
            }
            ++this->next_key;
        }
    }

public:
    using BaseHeap_ = detail::base_heap<T, Container, Compare>;

    template <typename VTy>
    requires std::constructible_from<value_type, VTy>
    constexpr indexed_d_ary_heap(std::initializer_list<VTy> init)
       : BaseHeap_(init.begin(), init.end()) { // Appends data only
        this->initialize_keys();
        this->build_heap();
    }

    //Delegate to the constructors of base_heap
    template<typename ...Arg>
    explicit constexpr indexed_d_ary_heap(Arg&&... args)
    : BaseHeap_(std::forward<Arg>(args)...) {
        this->initialize_keys();
        this->build_heap();
    }

    constexpr indexed_d_ary_heap(const indexed_d_ary_heap&) = default;
    constexpr indexed_d_ary_heap(indexed_d_ary_heap&&) = default;
    constexpr indexed_d_ary_heap& operator=(const indexed_d_ary_heap&) = default;
    constexpr indexed_d_ary_heap& operator=(indexed_d_ary_heap&&) = default;

    // Accessors
    // Non-const accessors are not provided
    // Methods concerning the top element (e.g. pop, top) are
    // noexcept with UB on empty heap. Other methods throw upon invalid heap states.
    [[nodiscard]] constexpr key_type top_key() const noexcept(noexcept(this->keys_[0])) {
        return this->keys_[0];
    }

    [[nodiscard]] constexpr const_reference at(const key_type key) const {
        auto it = this->positions_.find(key);
        if (it == this->positions_.end()) [[unlikely]] {
            throw std::out_of_range("Key not found");
        }
        return this->data_[it->second];
    }

    [[nodiscard]] constexpr const_reference operator[](const key_type key) const noexcept {
        // UB if key doesn't exist
        return this->data_[this->positions_.find(key)->second];
    }

    [[nodiscard]] constexpr bool contains(const key_type& key) const noexcept {
        return this->positions_.contains(key);
    }

    // Modifiers
    template <typename... Args>
    requires std::constructible_from<T, Args&&...>
    constexpr key_type emplace(Args&& ...args) {
        // Strong exception guarantee - emplace all or emplace none
        const auto key = this->next_key;
        const auto size = this->data_.size();
        this->insert_all_noheapify(key, size, std::forward<Args>(args)...);
        this->heapify_up(size);
        ++this->next_key;
        return key;
    }

    template <concepts::compatible_iterator<value_type> Iter,
              concepts::sentinel_or_iter<Iter> Sentinel,
              std::output_iterator<key_type> OpIter>
    constexpr void push_range(Iter first, Sentinel last, OpIter out) {
        // Basic exception safety in general - takes in as many elements as possible until it throws
        // Strong exception gurantee for each input - see the above
        if constexpr (std::random_access_iterator<Iter>) {
            const size_type n = this->size(), m = static_cast<size_type>(last - first);
            if constexpr (concepts::reservable_container<Container>) {
                this->data_.reserve(n + m);
            } if constexpr (concepts::reservable_container<key_storage>) {
                this->keys_.reserve(n + m);
            } if constexpr (concepts::reservable_container<position_storage>) {
                this->positions_.reserve(n + m);
            }

            auto logdn = [](const size_type x) -> size_type { // an approximate value of logd(n)
                if (x < d) return 1;
                if constexpr (d == 2) {
                    return static_cast<size_type>(std::bit_width(x) - 1);
                }
                return static_cast<size_type>(std::bit_width(x) / std::bit_width(d));
            };
            //rebuild if m >= (n / max(1, log2(n))), O(n + m)
            if (!n || m >= n / logdn(n)) {
                for (; first != last; ++first) {
                    const auto key = this->next_key;
                    const auto size = this->data_.size();
                    this->insert_all_noheapify(key, size, std::forward<decltype(*first)>(*first));
                    *out++ = this->next_key++;
                }
                this->build_heap();
                return;
            }
        }
        // Otherwise, or for non-random access iterator, heapify one-by-one, O(m log(n + m))
        size_type start_pos_ = this->size();
        for (; first != last; ++first, ++start_pos_) {
            const auto key = this->next_key;
            this->insert_all_noheapify(key, start_pos_, std::forward<decltype(*first)>(*first));
            this->heapify_up(start_pos_);
            *out++ = this->next_key++;
        }
    }

    template <typename Input>
    requires std::assignable_from<value_type&, Input&&>
    constexpr void update_top(Input&& value) {
        this->data_[0] = std::forward<Input>(value);
        this->heapify_down(0);
    }

    template <typename Input>
    requires std::assignable_from<value_type&, Input&&>
    constexpr void update_key(const key_type key, Input&& value) {
        auto it = this->positions_.find(key);
        if (it == this->positions_.end()) [[unlikely]] {
            throw std::out_of_range("Key not found");
        }
        const auto idx = it->second;
        this->data_[idx] = std::forward<Input>(value);
        this->update_at(idx);
    }

    template <typename Input>
    requires std::assignable_from<value_type&, Input&&>
    constexpr void unchecked_update_key(const key_type key, Input&& value) {
        const auto idx = this->positions_.find(key)->second;
        this->data_[idx] = std::forward<Input>(value);
        this->update_at(idx);
    }

    constexpr void pop() {
        this->erase_at(this->positions_.find(this->keys_[0]));
    }

    constexpr void erase(const key_type key) {
        auto it = this->positions_.find(key);
        if (it == this->positions.end()) [[unlikely]] {
            throw std::out_of_range("Key not found");
        }
        this->erase_at(it);
    }

    constexpr void unchecked_erase(const key_type key) {
        this->erase_at(this->positions_.find(key));
    }

    // Utilities
    constexpr void reserve(size_type n)
    requires concepts::reservable_container<Container> {
        BaseHeap_::reserve(n); // this->data_.reserve(n);
        if constexpr (concepts::reservable_container<key_storage>) {
            this->keys_.reserve(n);
        }
        if constexpr (concepts::reservable_container<position_storage>) {
            this->positions_.reserve(n);
        }
    }

    constexpr void swap(indexed_d_ary_heap& rhs)
    noexcept(std::is_nothrow_swappable_v<Container> &&
             std::is_nothrow_swappable_v<Compare> &&
             std::is_nothrow_swappable_v<key_storage> &&
             std::is_nothrow_swappable_v<position_storage>)
    requires std::is_empty_v<Compare> || std::swappable<Compare> {
        // concepts::container ensures other containers are swappable
        using std::swap;
        swap(this->data_, rhs.data_);
        swap(this->keys_, rhs.keys_);           // key_storage
        swap(this->positions_, rhs.positions_); // position_storage
        swap(this->next_key, rhs.next_key);   // key_type
        if constexpr (!std::is_empty_v<Compare>){
            swap(this->compare_, rhs.compare_);
        }
    }

    friend constexpr void swap(indexed_d_ary_heap& lhs, indexed_d_ary_heap& rhs)
    noexcept(noexcept(lhs.swap(rhs)))
    requires std::is_empty_v<Compare> || std::swappable<Compare> {
        lhs.swap(rhs);
    }

    constexpr void clear()
    noexcept(noexcept(this->data_.clear()) &&
             noexcept(this->keys_.clear()) &&
             noexcept(this->positions_.clear())) {
        this->data_.clear();
        this->keys_.clear();
        this->positions_.clear();
        this->next_key = key_type{};
    }

    // Note that if lhs.data_ == rhs.data_ and lhs.keys_ == rhs.keys_, lhs.positions_
    // and rhs.positions_ must be the same, so they are not compared
    friend constexpr bool operator== (const indexed_d_ary_heap& lhs, const indexed_d_ary_heap& rhs)
    noexcept(concepts::nothrow_equality_comparable<value_type> &&
             concepts::nothrow_equality_comparable<key_type>)
    requires concepts::equality_comparable<value_type> && concepts::equality_comparable<key_type> {
        return lhs.size() == rhs.size() &&
               std::ranges::equal(lhs.data_, rhs.data_) &&
               std::ranges::equal(lhs.keys_, rhs.keys_);
    }

    friend constexpr auto operator<=>(const indexed_d_ary_heap& lhs, const indexed_d_ary_heap& rhs)
    // requires both value_type and key_type to be noexcept comparable
    noexcept(((std::three_way_comparable<value_type> && concepts::nothrow_three_way_comparable<value_type>)
         || (!std::three_way_comparable<value_type> && concepts::nothrow_less_comparable<value_type>))
      &&    ((std::three_way_comparable<key_type> && concepts::nothrow_three_way_comparable<key_type>)
         || (!std::three_way_comparable<key_type> && concepts::nothrow_less_comparable<key_type>)))
    requires concepts::less_comparable<value_type> && concepts::less_comparable<key_type> {
        using common_ordering =
            std::conditional_t<(std::three_way_comparable<value_type> && std::three_way_comparable<key_type>),
                                std::common_comparison_category_t<
                                    std::compare_three_way_result_t<value_type>,
                                    std::compare_three_way_result_t<key_type>>,
                                std::weak_ordering>;

        auto cmp = []<typename Seq>(const Seq& lhs, const Seq& rhs) -> common_ordering {
            if constexpr (std::three_way_comparable<typename Seq::value_type>) {
                auto res = std::lexicographical_compare_three_way(
                    lhs.cbegin(), lhs.cend(),
                    rhs.cbegin(), rhs.cend());
                return static_cast<common_ordering>(res);
            }
            const auto sz = std::min(lhs.size(), rhs.size());
            for (size_type i = 0; i < sz; ++i) {
                if (lhs[i] < rhs[i]) {
                    return common_ordering::less;
                } if (rhs[i] < lhs[i]) {
                    return common_ordering::greater;
                }
            }
            // assumes equivalent if !(a < b) && !(b < a)
            return static_cast<common_ordering>(lhs.size() <=> rhs.size());
        };
        if (auto res = cmp(lhs.data_, rhs.data_); res != 0) {
            return res;
        }
        return cmp(lhs.keys_, rhs.keys_);
    }

};

template <typename T,
          typename Container = std::vector<T>,
          typename Compare = compare::less<>,
          typename KeyType = uint64_t,
          typename KeyStorage = std::vector<KeyType>>
using indexed_binary_heap = indexed_d_ary_heap<T, 2, Container, Compare, KeyType, KeyStorage>;

template <typename T,
          typename Container = std::vector<T>,
          typename Compare = compare::less<>,
          typename KeyType = uint64_t,
          typename KeyStorage = std::vector<KeyType>>
using indexed_ternary_heap = indexed_d_ary_heap<T, 3, Container, Compare, KeyType, KeyStorage>;

template <typename T,
          typename Container = std::vector<T>,
          typename KeyType = uint64_t,
          typename KeyStorage = std::vector<KeyType>>
using indexed_min_heap = indexed_d_ary_heap<T, 4, Container, compare::greater<>, KeyType, KeyStorage>;

template <typename T,
          typename Container = std::vector<T>,
          typename KeyType = uint64_t,
          typename KeyStorage = std::vector<KeyType>>
using indexed_max_heap = indexed_d_ary_heap<T, 4, Container, compare::less<>, KeyType, KeyStorage>;


template <typename T,
          std::size_t N,
          typename Compare = compare::less<>,
          typename KeyType = uint64_t>
using static_indexed_heap = indexed_d_ary_heap<T, 4, urlicht::inplace_vector<T, N>,
                                               Compare, KeyType, urlicht::inplace_vector<KeyType, N>>;

}
#endif //INDEXED_DHEAP_H
