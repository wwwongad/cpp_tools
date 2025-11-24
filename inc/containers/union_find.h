//
// Created by Peter on 10/4/2025.
//

#ifndef UNIONFIND_H
#define UNIONFIND_H
#pragma once
#include <concepts>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <functional>
#include <memory_resource>
#include <unordered_map>
#include <variant>

/* TODO: Heterogeneous Lookups to avoid implicit type cast*/

template<typename T>
concept Hashable = requires(T a) {
    { std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
};

// Defaults to std::vector for integral types; std::unordered_map otherwise.
// However, std::unordered_map can also be chosen for integral types
// To ensure that the size of an UnionFind is exactly the number of elements we explicitly inserted.
template <typename T, bool TrackSize = false, bool UsesMap = !std::integral<T>>
requires std::copyable<T> && std::equality_comparable<T> && Hashable<T>
class union_find {
public:
    using value_type = T;
    using size_type = std::size_t;
private:
    using ParentStorage = std::conditional_t<!UsesMap,
            std::vector<value_type>, std::unordered_map<value_type, value_type>>;

    using RankStorage = std::conditional_t<!UsesMap,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;

    using SizeStorage = std::conditional_t<!UsesMap,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;
public:
    using difference_type = std::ptrdiff_t;
    using container_type = ParentStorage;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using iterator = typename ParentStorage::iterator;
    using const_iterator = typename ParentStorage::const_iterator;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
private:

    mutable ParentStorage parents_;
    mutable RankStorage ranks_;
    mutable std::conditional_t<TrackSize, SizeStorage, std::monostate> sizes_;
    size_type set_count_{0};

    template <typename Storage>
    static auto& get_at(Storage& storage, const value_type& key) noexcept {
        if constexpr (!UsesMap) {
            return storage[key];
        }
        else {
            return storage.find(key)->second;
        }
    }

    template <typename Storage>
    static const auto& get_const(const Storage& storage, const value_type& key) noexcept {
        if constexpr (!UsesMap) {
            return storage[key];
        }
        else {
            return storage.find(key)->second;
        }
    }

    template <bool IsConst = false>
    const value_type& get_ensured_ancestor(const value_type& x) const noexcept {
        // Use pointers to avoid unnecessary copying
        const value_type* root = &x;
        while (true) {
            const value_type* parent = &get_const(parents_, *root);
            if (*parent == *root) {
                root = parent;
                break;
            }
            root = parent;
        }

        if constexpr (!IsConst) {
            value_type* next = &get_at(parents_, x);
            while (*next != *root) {
                value_type* curr = next;
                next = &get_at(parents_, *curr);
                *curr= *root;
            }
        }
        return *root;
    }

    void unite_ensured_root(const value_type& root_x, const value_type& root_y) noexcept {
        if (root_x == root_y) [[unlikely]] {
            return;
        }
        const size_type rank_x = get_const(ranks_, root_x);
        const size_type rank_y = get_const(ranks_, root_y);

        auto merge = [&](const value_type& root_a, const value_type& root_b) { //merge root_a to root_b
            if constexpr (TrackSize) {
                get_at(sizes_, root_b) += get_at(sizes_, root_a);
            }
            get_at(parents_, root_a) = root_b;
        };

        if (rank_x <= rank_y) {
            merge(root_x, root_y);
            if (rank_x == rank_y) ++get_at(ranks_, root_y);
        }
        else {
            merge(root_y, root_x);
        }
        --set_count_;
    }

public:
    // Constructors and Destructor
    // Default constructor - for std::unordered_map only
    union_find() requires (UsesMap) = default;

    // For std::vector
    explicit union_find(size_type n)
    requires (!UsesMap)
    : parents_(n), ranks_(n, 1), set_count_(n) {
        std::iota(parents_.begin(), parents_.end(), 0);
        if constexpr (TrackSize) {
            sizes_.assign(n, 1);
        }
    }

    // For std::unordered_map
    template <std::input_iterator InputIt>
    requires UsesMap && std::constructible_from<value_type, std::iter_reference_t<InputIt>>
    explicit union_find(InputIt first, InputIt last) {
        insert(first, last);
    }

    template <std::ranges::input_range Rng>
    requires UsesMap && std::constructible_from<value_type, std::iter_reference_t<Rng>>
    explicit union_find(Rng&& rng) {
        insert(std::forward<Rng>(rng));
    }

    union_find(const union_find& other) = default;
    union_find(union_find&& other) = default;
    union_find& operator=(const union_find& other) = default;
    union_find& operator=(union_find&& other) = default;
    ~union_find() = default;

    // Modifiers - for std::unordered_map only
    template <typename Val>
    requires std::convertible_to<Val, value_type>
    void insert(Val&& key) requires (UsesMap) {
        if (parents_.contains(key)) return;
        // Basic exception guarantee
        parents_.emplace(key, key);
        if constexpr (TrackSize) {
            sizes_.emplace(key, 1);
        }
        ranks_.emplace(std::forward<Val>(key), 1);
        ++set_count_;
    }

    template <std::input_iterator InputIt>
    requires UsesMap && std::constructible_from<value_type, std::iter_reference_t<InputIt>>
    void insert(InputIt first, InputIt last) {
        if constexpr (std::random_access_iterator<InputIt>) {
            const auto m = this->size();
            const auto n = static_cast<size_type>(last - first);
            parents_.reserve(m + n);
            ranks_.reserve(m + n);
            if constexpr (TrackSize) {
                sizes_.reserve(m + n);
            }
        }
        for (; first != last; ++first) {
            insert(std::forward<decltype(*first)>(*first));
        }
    }

    template <std::ranges::range Rng>
    requires UsesMap && std::constructible_from<value_type, std::ranges::range_reference_t<Rng>>
    void insert(Rng&& rng) {
        if constexpr (std::is_rvalue_reference_v<decltype(rng)>) {
            this->insert(std::make_move_iterator(std::ranges::begin(rng)),
                    std::make_move_iterator(std::ranges::end(rng)));
        } else {
            this->insert(std::ranges::begin(rng), std::ranges::end(rng));
        }
    }

    // Modifier for std::vector
    void resize(size_type new_size)
    requires (!UsesMap) {
        const auto old_size = parents_.size();
        // Disjoint set union does not support detaching elements from a set
        if (new_size <= old_size) return;

        //Strong exception guarantee
        bool s_ok = false, r_ok = false;
        try {
            if constexpr (TrackSize) {
                sizes_.resize(new_size); s_ok = true;
            }
            ranks_.resize(new_size); r_ok = true;
            parents_.resize(new_size);
        }
        catch (...) {
            if constexpr (TrackSize) {
                if (s_ok) sizes_.resize(old_size);
            }
            if (r_ok) ranks_.resize(old_size);
            // Note that we would never have to resize parents_
            throw;
        }
        // The following are all noexcept operations
        if constexpr (TrackSize) {
            std::fill(sizes_.begin() + old_size, sizes_.begin() + new_size, 1);
        }
        std::fill(ranks_.begin() + old_size, ranks_.begin() + new_size, 1);
        std::iota(parents_.begin() + old_size, parents_.begin() + new_size, old_size);
        set_count_ += new_size - old_size;
    }

    // Core methods
    [[nodiscard]] const_reference find(const value_type& x) {
        if (!contains(x) /* for both std::vector and std::unordered_map */) {
            if constexpr (UsesMap) this->insert(x);
            else this->resize(x + 1);
        }
        return get_ensured_ancestor<false /* does path compression */ >(x);
    }

    [[nodiscard]] const_reference find_existing(const value_type& x) {
        // throw upon non-existent value in safe method
        if (!contains(x))
            throw std::runtime_error("UnionFind::find_existing() called on non-existent value");

        return get_ensured_ancestor<false /* does path compression */ >(x);
    }

    [[nodiscard]] const_reference unchecked_find_existing(const value_type& x) noexcept {
        return get_ensured_ancestor<false /* does path compression */ >(x);
    }

    [[nodiscard]] const_reference find_const(const value_type& x) const {
        // throw upon non-existent value in safe method
        if (!contains(x))
            throw std::runtime_error("UnionFind::find_const() called on non-existent value");

        return get_ensured_ancestor<true /* omits path compression */ >(x);
    }

    [[nodiscard]] const_reference unchecked_find_const(const value_type& x) const noexcept {
        return get_ensured_ancestor<true /* omits path compression */ >(x);
    }

    void unite_new(const value_type& x, const value_type& y) {
        unite_ensured_root(find(x), find(y));
    }

    void unite_existing(const value_type& x, const value_type& y) {
        unite_ensured_root(find_existing(x), find_existing(y));
    }

    void unchecked_unite_existing(const value_type& x, const value_type& y) noexcept {
        unite_ensured_root(unchecked_find_existing(x), unchecked_find_existing(y));
    }

    bool try_unite_existing(const value_type& x, const value_type& y) noexcept {
        if (!contains(x) || !contains(y)) {
            return false;
        }
        unite_ensured_root(unchecked_find_existing(x), unchecked_find_existing(y));
        return true;
    }

    // Query Methods
    // Iterator
    [[nodiscard]] const_iterator begin() const noexcept { return parents_.begin(); }
    [[nodiscard]] const_iterator end() const noexcept { return parents_.end(); }
    [[nodiscard]] const_reverse_iterator rbegin() const noexcept { return std::reverse_iterator{ begin() }; }
    [[nodiscard]] const_reverse_iterator rend() const noexcept { return std::reverse_iterator{ end() }; }
    [[nodiscard]] const_iterator cbegin() const noexcept { return parents_.cbegin(); }
    [[nodiscard]] const_iterator cend() const noexcept { return parents_.cend(); }
    [[nodiscard]] const_reverse_iterator crbegin() const noexcept { return std::reverse_iterator{ cend() }; }
    [[nodiscard]] const_reverse_iterator crend() const noexcept { return std::reverse_iterator{ cbegin() }; }

    // Accessors
    [[nodiscard]] const_pointer data() const noexcept requires (!UsesMap) { return parents_.data(); }
    [[nodiscard]] const_reference front() const noexcept requires (!UsesMap) { return parents_.front();}
    [[nodiscard]] const_reference back() const noexcept requires (!UsesMap) { return parents_.back(); }

    // Capacity
    [[nodiscard]] size_type size() const noexcept {
        return parents_.size();
    }

    [[nodiscard]] size_type max_size() const noexcept {
        return parents_.max_size();
    }

    [[nodiscard]] size_type capacity() const noexcept
    requires (!UsesMap) {
        return parents_.capacity();
    }

    [[nodiscard]] bool empty() const noexcept {
        return parents_.empty();
    }

    void reserve(size_type n) {
        if (n < this->size()) {
            return;
        }
        parents_.reserve(n);
        ranks_.reserve(n);
        if constexpr (TrackSize) {
            sizes_.reserve(n);
        }
    }

    // UnionFind specific
    [[nodiscard]] bool same_set(const value_type& x, const value_type& y) const noexcept {
        return this->contains(x) && this->contains(y) && find_const(x) == find_const(y);
    }

    [[nodiscard]] bool is_root(const value_type& x) const noexcept {
        if (!this->contains(x)) {
            return false;
        }
        return x == get_const(parents_, x);
    }

    [[nodiscard]] size_type set_rank(const value_type& x) const noexcept {
        if (!this->contains(x)) {
            return 0U;
        }
        return get_const(ranks_, find_const(x));
    }

    [[nodiscard]] size_type set_size(const value_type& x) const noexcept
    requires TrackSize {
        if (!this->contains(x)) {
            return 0U;
        }
        return get_const(sizes_, find_const(x));
    }

    [[nodiscard]] size_type set_count() const noexcept {
        return set_count_;
    }

    [[nodiscard]] bool contains(const value_type& x) const noexcept {
        if constexpr (!UsesMap) {
            return x >= 0 && x < this->size();
        }
        else return parents_.contains(x);
    }

    // Utility
    void clear() noexcept {
        parents_.clear();
        ranks_.clear();
        if constexpr (TrackSize) {
            sizes_.clear();
        }
        set_count_ = 0U;
    }

    void swap(union_find& other) noexcept(std::is_nothrow_swappable_v<ParentStorage>) {
        using std::swap;
        swap(parents_, other.parents_);
        swap(ranks_, other.ranks_);
        if constexpr (TrackSize) {
            swap(sizes_, other.sizes_);
        }
        swap(set_count_, other.set_count_);
    }

    bool operator==(const union_find& other) const noexcept
    requires std::equality_comparable<ParentStorage>{
        return this->set_count_ == other.set_count_ // for early termination only
             && this->parents_ == other.parents_;
    }

    friend bool operator==(const union_find& lhs, const union_find& rhs) noexcept {
        return lhs.operator==(rhs);
    }

    friend void swap(union_find& lhs, union_find& rhs) noexcept(noexcept(lhs.swap(rhs))) {
        lhs.swap(rhs);
    }
};



#endif //UNIONFIND_H

