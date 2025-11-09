//
// Created by Peter on 10/4/2025.
//

#ifndef UNIONFIND_H
#define UNIONFIND_H
#include <concepts>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <functional>
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
class UnionFind {
public:
    using value_type = T;
    using const_reference = const T&;
    using size_type = std::size_t;

private:
    using ParentStorage = std::conditional_t<!UsesMap,
            std::vector<value_type>, std::unordered_map<value_type, value_type>>;

    using RankStorage = std::conditional_t<!UsesMap,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;

    using SizeStorage = std::conditional_t<!UsesMap,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;

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

    template <bool IsBitwiseConst = false>
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

        if constexpr (!IsBitwiseConst) {
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
        if (root_x == root_y) {
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

        if (rank_x < rank_y) {
            merge(root_x, root_y);
        }
        else if (rank_x > rank_y) {
            merge(root_y, root_x);
        }
        else {
            merge(root_x, root_y);
            ++get_at(ranks_, root_y);
        }
        --set_count_;
    }

public:
    // Constructors and Destructor
    // Default constructor - for std::unordered_map only
    UnionFind() requires UsesMap = default;

    // For std::vector
    explicit UnionFind(size_type n)
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
    explicit UnionFind(InputIt first, InputIt last) {
        insert(first, last);
    }

    UnionFind(const UnionFind& other) = default;
    UnionFind(UnionFind&& other) = default;
    UnionFind& operator=(const UnionFind& other) = default;
    UnionFind& operator=(UnionFind&& other) = default;
    ~UnionFind() = default;

    // Modifiers - for std::unordered_map only
    template <typename Val>
    requires UsesMap && std::constructible_from<value_type, Val>
    void insert(Val&& x) {
        if (parents_.contains(x)) return;
        if constexpr (TrackSize) sizes_[x] = 1;
        ranks_[x] = 1;
        parents_[x] = std::forward<Val>(x);
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
        insert(std::ranges::begin(rng), std::ranges::end(rng));
    }


    // Modifier for std::vector
    void resize(size_type new_size)
    requires !UsesMap {
        const auto old_size = parents_.size();
        // Disjoint set union does not support detaching elements from a set
        if (new_size <= old_size) return;

        parents_.resize(new_size);
        std::iota(parents_.begin() + old_size, parents_.begin() + new_size, old_size);

        ranks_.resize(new_size);
        std::fill(ranks_.begin() + old_size, ranks_.begin() + new_size, 1);

        if constexpr (TrackSize) {
            sizes_.resize(new_size);
            std::fill(sizes_.begin() + old_size, sizes_.begin() + new_size, 1);
        }
        set_count_ += new_size - old_size;
    }

    // Core methods
    [[nodiscard]] const_reference find(const value_type& x) {
        if (!contains(x) /* for both std::vector and std::unordered_map */) {
            insert(x);
        }
        return get_ensured_ancestor<false>(x);
    }

    template <bool Unsafe = true>
    [[nodiscard]] const_reference find_existing(const value_type& x) noexcept(Unsafe) {
        if constexpr (!Unsafe) { // throw upon non-existent value in safe method
            if (!contains(x))
                throw std::runtime_error("UnionFind::find_existing() called on non-existent value");
        }
        return get_ensured_ancestor<false/* does path compression */>(x);
    }

    template <bool Unsafe = true>
    [[nodiscard]] const_reference find_const(const value_type& x) const noexcept(Unsafe) {
        if constexpr (!Unsafe) { // throw upon non-existent value in safe method
            if (!contains(x))
                throw std::runtime_error("UnionFind::find_const() called on non-existent value");
        }
        return get_ensured_ancestor<true/* omits path compression */>(x);
    }

    void unite_new(const value_type& x, const value_type& y) {
        unite_ensured_root(find(x), find(y));
    }

    template<bool Unsafe = true>
    void unite_existing(const value_type& x, const value_type& y) noexcept(Unsafe) {
        unite_ensured_root(find_existing<Unsafe>(x), find_existing<Unsafe>(y));
    }

    bool try_unite(const value_type& x, const value_type& y) {
        if (!contains(x) || !contains(y)) {
            return false;
        }
        unite_ensured_root(find_existing(x), find_existing(y));
        return true;
    }

    // Query Methods
    // Capacity
    [[nodiscard]] size_type size() const noexcept {
        return parents_.size();
    }

    [[nodiscard]] size_type max_size() const noexcept {
        return parents_.max_size();
    }

    [[nodiscard]] size_type capacity() const noexcept
    requires requires { parents_.capacity(); } { // Because std::unordered_map does not have capacity() method
        return parents_.capacity();
    }

    [[nodiscard]] bool empty() const noexcept {
        return parents_.empty();
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

    void swap(UnionFind& other) noexcept {
        using std::swap;
        swap(parents_, other.parents_);
        swap(ranks_, other.ranks_);
        if constexpr (TrackSize) {
            swap(sizes_, other.sizes_);
        }
        swap(set_count_, other.set_count_);
    }

};

template <typename T, bool TrackSize, bool UsesMap>
void swap(UnionFind<T, TrackSize, UsesMap>& lhs,
            UnionFind<T, TrackSize, UsesMap>& rhs) noexcept {
    lhs.swap(rhs);
}

#endif //UNIONFIND_H

