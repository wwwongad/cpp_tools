#ifndef INDEXED_DHEAP_H
#define INDEXED_DHEAP_H
#pragma once
#include <cstdint>
#include <stdexcept>
#include <algorithm> // for std::min
#include <bit>       // for std::bit_width
#include <iterator>  // for std::distance, std::make_move_iterator
#include <utility>   // for std::move, std::forward
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

    [[nodiscard]] static constexpr size_type parent(const size_t i) noexcept { return (i - 1) / d; }
    [[nodiscard]] static constexpr size_type left(const size_t i) noexcept { return d * i + 1; }

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

    constexpr void initialize_keys() { // Initialize keys and positions after the values are initialized
        if constexpr (concepts::reservable_container<position_storage>) {
            this->positions_.reserve(this->data_.size());
        }
        if constexpr (concepts::reservable_container<key_storage>) {
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

    template <concepts::compatible_iterator<value_type> Iter,
              concepts::sentinel_or_iter<Iter> Sentinel>
    constexpr size_type push_range_impl(Iter first, Sentinel last) {
        // Basic exception safety in general - takes in as many elements as possible until it throws
        // Strong exception gurantee for each input - see the above
        const auto n = this->size();
        auto curr_pos = n;
        size_type m = 0;

        if constexpr (std::forward_iterator<Iter>) {
            m = static_cast<size_type>(std::distance(first, last));
            // Reserve if possible
            if constexpr (concepts::reservable_container<Container>) {
                this->data_.reserve(n + m);
            }
            if constexpr (concepts::reservable_container<key_storage>) {
                this->keys_.reserve(n + m);
            }
            if constexpr (concepts::reservable_container<position_storage>) {
                this->positions_.reserve(n + m);
            }

            auto logdn = [](const size_type x) -> size_type { // an approximate value of logd(n) in O(1)
                if (x < d) {
                    return 1;
                }
                if constexpr (d == 2) {
                    return static_cast<size_type>(std::bit_width(x) - 1);
                } else {
                    return static_cast<size_type>(std::bit_width(x) / std::bit_width(d));
                }
            };

            //rebuild if m >= (n / max(1, log2(n))), O(n + m)
            if (!n || m >= n / logdn(n)) {
                for (; first != last; ++first, ++curr_pos) {
                    const auto key = this->next_key;
                    this->insert_all_noheapify(key, curr_pos, *first);
                    ++this->next_key;
                }
                this->build_heap();
            } else {
                // Otherwise, heapify one-by-one, O(m log(n + m))
                for (; first != last; ++first, ++curr_pos) {
                    const auto key = this->next_key;
                    this->insert_all_noheapify(key, curr_pos, *first);
                    this->heapify_up(curr_pos);
                    ++this->next_key;
                }
            }
        } else {  // if constexpr forward iterator
            // Fall back to per-element insertion for input iterators
            for (; first != last; ++first, ++curr_pos) {
                const auto key = this->next_key;
                this->insert_all_noheapify(key, curr_pos, *first);
                this->heapify_up(curr_pos);
                ++this->next_key;
                ++m;
            }
        }
        return m;
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
        // For exception guarantee, see the above
        const key_type first_key = this->next_key;
        const size_type m = push_range_impl(first, last);

        for (key_type key = first_key; key < first_key + m; ++key) {
            *out++ = key;
        }
    }

    template <concepts::compatible_iterator<value_type> Iter,
              concepts::sentinel_or_iter<Iter> Sentinel>
    constexpr std::pair<key_type, size_type> push_range(Iter first, Sentinel last) {
        // For exception guarantee, see the above
        const key_type first_key = this->next_key;
        const size_type m = push_range_impl(first, last);

        return {first_key, m};
    }

    template <concepts::compatible_range<value_type> Rng,
              std::output_iterator<key_type> OpIter>
    constexpr void push_range(Rng&& rng, OpIter out) {
        if constexpr (concepts::rvalue_range<Rng&&>) {
            this->push_range(
                    std::make_move_iterator(std::ranges::begin(rng)),
                    std::make_move_iterator(std::ranges::end(rng)), out);
        } else {
            this->push_range(std::ranges::begin(rng), std::ranges::end(rng), out);
        }
    }

    template <concepts::compatible_range<value_type> Rng>
    constexpr std::pair<key_type, size_type> push_range(Rng&& rng) {
        if constexpr (concepts::rvalue_range<Rng&&>) {
            return this->push_range(
                    std::make_move_iterator(std::ranges::begin(rng)),
                    std::make_move_iterator(std::ranges::end(rng)));
        } else {
            return this->push_range(std::ranges::begin(rng), std::ranges::end(rng));
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

};

// Type alias
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
          typename Compare = compare::less<>,
          typename KeyType = uint64_t,
          typename KeyStorage = std::vector<KeyType>>
using indexed_quandary_heap = indexed_d_ary_heap<T, 4, Container, Compare, KeyType, KeyStorage>;

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
using static_indexed_heap = indexed_d_ary_heap<T, 4, inplace_vector<T, N>,
                                               Compare, KeyType, inplace_vector<KeyType, N>>;

// CTAD
template <typename T>
indexed_d_ary_heap(std::initializer_list<T>) -> indexed_d_ary_heap<T>;

template <concepts::random_access_container Container>
indexed_d_ary_heap(Container) -> indexed_d_ary_heap<typename Container::value_type>;

} // namespace urlicht
#endif //INDEXED_DHEAP_H
