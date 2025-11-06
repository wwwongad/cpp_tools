
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

template <typename T, bool TrackSize = false>
requires std::copyable<T> && std::equality_comparable<T> && Hashable<T>
class UnionFind {
public:
    using value_type = T;
    using size_type = std::size_t;

private:
    using ParentStorage = std::conditional_t<std::integral<value_type>,
            std::vector<value_type>, std::unordered_map<value_type, value_type>>;

    using RankStorage = std::conditional_t<std::integral<value_type>,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;

    using SizeStorage = std::conditional_t<std::integral<value_type>,
            std::vector<size_type>, std::unordered_map<value_type, size_type>>;

    mutable ParentStorage parents_;
    mutable RankStorage ranks_;
    mutable std::conditional_t<TrackSize, SizeStorage, std::monostate> sizes_;
    size_type set_count_{0};

    template <typename Map>
    static auto& map_access(Map& map, const value_type& key) noexcept {
        if constexpr (std::integral<value_type>) {
            return map[key];
        }
        else {
            return map.find(key)->second;
        }
    }

    size_type& get_size(const value_type& x) const noexcept
    requires TrackSize {
        return map_access(sizes_, x);
    }

    size_type& get_rank(const value_type& x) const noexcept {
        return map_access(ranks_, x);
    }

    value_type& get_direct_parent(const value_type& x) const noexcept {
        return map_access(parents_, x);
    }

    const value_type& get_ensured_ancestor(const value_type& x) const noexcept {
        const value_type* root = &x; // Avoid copies by keeping track of pointers
        while (true) {
            const value_type& parent = get_direct_parent(*root);
            if (parent == *root) {
                root = &parent;
                break;
            }
            root = &parent;
        }

        value_type curr{x};
        while (curr != *root) {
            value_type& curr_parent_at = get_direct_parent(curr);
            curr = std::move(curr_parent_at);  // Copy before modifying
            curr_parent_at = *root;
        }
        return *root;
    }

    void unite_ensured_root(const value_type& root_x, const value_type& root_y) noexcept {
        if (root_x == root_y) {
            return;
        }
        const size_type rank_x = get_rank(root_x);
        const size_type rank_y = get_rank(root_y);

        auto merge = [&](const value_type& root_a, const value_type& root_b) {
            get_direct_parent(root_a) = root_b;
            if constexpr (TrackSize) {
                get_size(root_b) += get_size(root_a);
            }
        };

        if (rank_x < rank_y) merge(root_x, root_y);
        else if (rank_x > rank_y) merge(root_y, root_x);
        else {
            merge(root_x, root_y);
            ++get_rank(root_y);
        }
        --set_count_;
    }

public:
    // Constructors and Destructor
    UnionFind() requires (!std::integral<value_type>) = default;

    explicit UnionFind(size_type n)
    requires std::integral<value_type>
    : parents_(n), ranks_(n, 1), set_count_(n) {
        std::iota(parents_.begin(), parents_.end(), 0);
        if constexpr (TrackSize) {
            sizes_.assign(n, 1);
        }
    }

    template <std::input_iterator InputIt>
    requires (!std::integral<value_type>)
            && std::constructible_from<value_type, std::iter_reference_t<InputIt>>
    UnionFind(InputIt first, InputIt last) {
        for (; first != last; ++first) {
            insert(*first);
        }
    }

    UnionFind(const UnionFind& other) = default;
    UnionFind(UnionFind&& other) = default;
    UnionFind& operator=(const UnionFind& other) = default;
    UnionFind& operator=(UnionFind&& other) = default;
    ~UnionFind() = default;

    // Modifiers - for non-integral types only
    void insert(const value_type& x) requires (!std::integral<value_type>) {
        if (parents_.contains(x)) return;
        parents_[x] = x;
        ranks_[x] = 1;
        if constexpr (TrackSize) sizes_[x] = 1;
        ++set_count_;
    }

    template <std::ranges::range Rng>
    requires (!std::integral<value_type>)
            && std::constructible_from<value_type, std::ranges::range_reference_t<Rng>>
    void insert(Rng&& rng) {
        if constexpr (requires { std::ranges::size(rng) }) {
            const auto m = this->size();
            const auto n = static_cast<size_type>(std::range::size(rng));
            parents_.reserve(m + n);
            ranks_.reserve(m + n);
            if constexpr (TrackSize) {
                sizes_.reserve(m + n);
            }
        }
        for (auto&& x : rng) {
            insert(x);
        }
    }

    template <std::input_iterator InputIt>
    requires (!std::integral<value_type>)
            && std::constructible_from<value_type, std::iter_reference_t<InputIt>>
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
            insert(*first);
        }
    }

    // Modifier for integral types
    void resize(size_type new_size)
    requires std::integral<value_type> {
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
    [[nodiscard]] const value_type& find(const value_type& x) {
        if constexpr (std::integral<value_type>) {
            if (x >= parents_.size()) this->resize(x + 1);
        }
        else {
            if (!contains(x)) insert(x);
        }
        return get_ensured_ancestor(x);
    }

    [[nodiscard]] const value_type& find_existing(const value_type& x) const noexcept {
        return get_ensured_ancestor(x);
    }

    [[nodiscard]] std::optional<value_type> try_find(const value_type& x) const noexcept {
        if constexpr (std::integral<value_type>) {
            if (x >= parents_.size()) return std::nullopt;
        }
        else {
            if (!parents_.contains(x)) return std::nullopt;
        }
        return get_ensured_ancestor(x);
    }

    void unite_new(const value_type& x, const value_type& y) {
        unite_ensured_root(find_ensured(y), find_ensured(y));
    }

    void unite_existing(const value_type& x, const value_type& y)  {
        unite_ensured_root(find_existing(x), find_existing(y));
    }

    // Query Methods
    [[nodiscard]] bool same_set(const value_type& x, const value_type& y) const noexcept {
        const auto root_x = try_find(x);
        const auto root_y = try_find(y);
        return root_x != std::nullopt && root_x == root_y;
    }

    [[nodiscard]] size_type size() const noexcept {
        return parents_.size();
    }

    [[nodiscard]] size_type set_rank(const value_type& x) const noexcept {
        auto root = try_find(x);
        if (root == std::nullopt) {
            return 0U;
        }
        if constexpr (!std::integral<value_type>) {
            return ranks_.find(root.value())->second;
        }
        else return ranks_[root.value()];
    }

    [[nodiscard]] size_type set_size(const value_type& x) const noexcept
    requires TrackSize {
        auto root = try_find(x);
        if (root == std::nullopt) {
            return 0U;
        }
        if constexpr (!std::integral<value_type>) {
            return sizes_.find(root.value())->second;
        }
        else return sizes_[root.value()];
    }

    [[nodiscard]] bool empty() const noexcept {
        return parents_.empty();
    }

    [[nodiscard]] size_type set_count() const noexcept {
        return set_count_;
    }

    [[nodiscard]] bool contains(const value_type& x) const noexcept {
        if constexpr (std::integral<value_type>) {
            return x < this->size();
        }
        else return parents_.contains(x);
    }

    void clear() noexcept {
        parents_.clear();
        ranks_.clear();
        if constexpr (TrackSize) {
            sizes_.clear();
        }
        set_count_ = 0U;
    }
};

#endif //UNIONFIND_H

