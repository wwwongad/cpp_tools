#ifndef UNION_FIND_H
#define UNION_FIND_H
#include <concepts>
#include <unordered_map>
#include <variant>
#include <algorithm>
#include <numeric>
#include <vector>
#include "urlicht/concepts_utility.h"
#include "urlicht/container/inplace_vector.h"

namespace urlicht {
    namespace detail {
        /********************** UF CONCEPTS CHECKS AND DEFAULT TYPES *********************/

        // Parent storage can be either unordered_map or random_access_container.
        template <typename Cont, typename T>
        concept valid_parent_storage =
            (concepts::unordered_map<Cont> &&
                std::same_as<typename Cont::key_type, typename Cont::mapped_type> &&
                std::same_as<typename Cont::key_type, T>)
        // random_access_container -> value type must be unsigned integral
        ||  (concepts::random_access_container<Cont> &&
                std::unsigned_integral<T> &&
                std::same_as<typename Cont::value_type, T>);

        template <typename T>
        using default_parent_storage_t =
            std::conditional_t<std::unsigned_integral<T>, std::vector<T>, std::unordered_map<T, T>>;

        // If TrackSize is false, then size storage can be whatever type.
        // Otherwise, size storage must match parent storage
        template <typename SizeStorage, typename T, bool TrackSize, typename ParentStorage>
        concept valid_size_storage =
            !TrackSize || (
                (concepts::unordered_map<ParentStorage> && concepts::unordered_map<SizeStorage> &&
                 std::same_as<typename SizeStorage::key_type, T> &&
                 std::unsigned_integral<typename SizeStorage::mapped_type>)
            ||  (concepts::random_access_container<ParentStorage> &&
                 concepts::random_access_container<SizeStorage> &&
                 std::unsigned_integral<typename SizeStorage::value_type>)
            );

        // Default size storage binds mapped/value types to ParentStorage::size_type.
        // For maps: std::unordered_map<T, size_type>; for arrays: std::vector<size_type>.
        template <typename T, bool TrackSize, typename ParentStorage>
        using default_size_storage_t =
            std::conditional_t<TrackSize,
                std::conditional_t<concepts::unordered_map<ParentStorage>,
                    std::unordered_map<T, typename ParentStorage::size_type>,
                    std::vector<typename ParentStorage::size_type>>,
                std::monostate>;

        // Optimizes size storage away when TrackSize is false
        template <typename T, bool TrackSize, typename SizeStorage>
        using adaptive_size_storage_t = std::conditional_t<TrackSize, SizeStorage, std::monostate>;

        /******************************* SYNOPSIS *******************************/
        /*   Common methods provided by the base classes:  */
        // const_reference get_ensured_root(const_reference) const          (protected)
        // const_reference get_ensured_root_and_compress(const_reference)   (protected)
        // void unite_existing_roots(const_reference, const_reference)      (protected)
        // bool contains(const_reference) const noexcept
        // size_type set_size(const_reference) const noexcept
        // bool is_root(const_reference) const noexcept
        // void reset_all()

        /*   Map-based union-find specific methods:   */
        // bool try_emplace(Key&&)
        // void insert(InputIt, Sentinel)
        // void insert(std::initializer_list<U>)
        // void insert_range(Rng&&)

        /*   Array-based union-find specific methods:   */
        // void resize(size_type)

        /*   Methods provided by the derived class:   */
        // const_reference find_or_insert(const_reference)
        // const_reference unchecked_find(const_reference)
        // const_reference unchecked_find(const_reference) const
        // void unite_new(const_reference, const_reference)
        // bool try_unite(const_reference, const_reference)
        // void unchecked_unite(const_reference, const_reference)
        // size_type set_count() const noexcept
        // bool same_set(const_reference, const_reference) const noexcept
        // Iterators - const_iterator only
        // Capecity
        // void clear()
        // void swap()


        /************************** IMPL FOR MAP-BASED UF *************************/

        template <
            typename T,
            bool TrackSize,
            typename ParentStorage,
            typename SizeStorage,
            typename KeyEqual>
        class map_uf_base_ {
        public:
            // Necessary ones only. Others are defined in the derived union-find class
            using parent_storage = ParentStorage;
            using size_storage = adaptive_size_storage_t<T, TrackSize, SizeStorage>;
            using value_type = T;
            using size_type = typename parent_storage::size_type;
            using key_equal = KeyEqual;
        protected:
            parent_storage parents_{};
            [[no_unique_address]] size_storage sizes_{}; // May be std::monostate
            size_type set_count_{0U};
            [[no_unique_address]] key_equal key_equal_{};

            [[nodiscard]]   // UB if val does not exist
            constexpr const value_type& get_ensured_root(const value_type& val) const {
                const value_type* prev = &val;
                typename parent_storage::const_iterator next;
                // Zero extra copies
                while (true) {
                    next = this->parents_.find(*prev);
                    if (this->key_equal_(next->second, *prev)) {
                        return next->second;
                    }
                    prev = &next->second;
                }
            }

            [[nodiscard]]
            constexpr const value_type& get_ensured_root_and_compress(const value_type& val) {
                const auto& root = get_ensured_root(val);
                auto next = this->parents_.find(val);
                while (!this->key_equal_(next->second, root)) {
                    auto curr = next;
                    next = this->parents_.find(next->second);
                    curr->second = root;    // Copies only occur here
                }
                return root;
            }

            constexpr void unite_existing_roots(const value_type& root_x, const value_type& root_y) {
                if (this->key_equal_(root_x, root_y)) [[unlikely]] {
                    return;
                }
                if constexpr (TrackSize) {
                    auto x_size_it = this->sizes_.find(root_x);
                    auto y_size_it = this->sizes_.find(root_y);
                    if (x_size_it->second > y_size_it->second) {
                        // If the set x belongs to is larger, merge the set of y to it
                        this->parents_.find(root_y)->second = root_x;
                        x_size_it->second += y_size_it->second;
                    } else {
                        // Otherwise, merge the set of x to the set of y
                        this->parents_.find(root_x)->second = root_y;
                        y_size_it->second += x_size_it->second;
                    }
                } else {
                    this->parents_.find(root_x)->second = root_y;
                }
                --this->set_count_;
            }

        public:
            /**************** Map-based union-find specific methods *****************/

            constexpr map_uf_base_() noexcept requires std::default_initializable<key_equal> = default;

            template <concepts::compatible_iterator<value_type> Iter,
                      concepts::sentinel_or_iter<Iter> Sentinel>
            constexpr map_uf_base_(Iter first, Sentinel last) {
                this->insert(first, last);
            }

            template <concepts::compatible_range<value_type> Rng>
            explicit constexpr map_uf_base_(Rng&& rng) {
                this->insert_range(std::forward<Rng>(rng));
            }

            constexpr map_uf_base_(const map_uf_base_&) = default;

            constexpr map_uf_base_(map_uf_base_&&)
            noexcept(
                     std::is_nothrow_move_constructible_v<parent_storage> &&
                     std::is_nothrow_move_constructible_v<size_storage> &&
                     std::is_nothrow_move_constructible_v<key_equal>
            ) = default;

            constexpr map_uf_base_& operator=(const map_uf_base_&) = default;

            constexpr map_uf_base_& operator=(map_uf_base_&&)
            noexcept(
                     std::is_nothrow_move_assignable_v<parent_storage> &&
                     std::is_nothrow_move_assignable_v<size_storage> &&
                     std::is_nothrow_move_assignable_v<key_equal>
            ) = default;

            constexpr ~map_uf_base_() = default;


            // Without size tracking
            template <typename Key>
            requires (!TrackSize) && std::constructible_from<value_type, Key&&>
            constexpr bool try_emplace(Key&& key) {
                const auto [it, inserted] = this->parents_.try_emplace(key, key);
                if (!inserted) {
                    return false;
                }
                ++this->set_count_;
                return true;
            }

            // With size tracking
            template <typename Key>
            requires TrackSize && std::constructible_from<value_type, Key&&>
            constexpr bool try_emplace(Key&& key) {
                // Strong exception guarentee
                bool p_done = false;
                typename parent_storage::iterator p_it;
                try {
                    bool inserted = false;
                    std::tie(p_it, inserted) = this->parents_.try_emplace(key, key);
                    if (!inserted) {
                        return false; // The element already exists
                    }
                    p_done = true;
                    this->sizes_.try_emplace(std::forward<Key>(key), 1);
                } catch (...) {
                    if (p_done) this->parents_.erase(p_it);  // This is noexcept
                    throw;
                }
                ++this->set_count_;
                return true;
            }

            template <concepts::compatible_iterator<value_type> Iter,
                      concepts::sentinel_or_iter<Iter> Sentinel>
            constexpr void insert(Iter first, Sentinel last) {
                if constexpr (std::random_access_iterator<Iter>) {
                    const auto m = this->parents_.size();
                    const auto n = static_cast<size_type>(std::distance(first, last));
                    this->parents_.reserve(m + n);
                    if constexpr (TrackSize) {
                        this->sizes_.reserve(m + n);
                    }
                }
                for (; first != last; ++first) {
                    this->try_emplace(*first);
                }
            }

            template <typename VTy>
            requires std::constructible_from<value_type, VTy>
            constexpr void insert(std::initializer_list<VTy> li) {
                this->insert(li.begin(), li.end());
            }

            template <concepts::compatible_range<value_type> Rng>
            constexpr void insert_range(Rng&& rng) {
                if constexpr (concepts::rvalue_range<Rng&&>) {
                    this->insert(std::make_move_iterator(std::ranges::begin(rng)),
                                 std::make_move_iterator(std::ranges::end(rng)));
                } else {
                    this->insert(std::ranges::begin(rng), std::ranges::end(rng));
                }
            }

            [[nodiscard]] constexpr bool contains(const value_type& x) const noexcept {
                return this->parents_.contains(x);
            }

            [[nodiscard]] constexpr size_type set_size(const value_type& x) const noexcept
            requires TrackSize {
                if (!this->contains(x)) {
                    return 0U;
                }
                return this->sizes_.find(this->get_ensured_root(x))->second;
            }

            [[nodiscard]] constexpr bool is_root(const value_type& x) const
            noexcept(std::is_nothrow_invocable_v<key_equal, const value_type&, const value_type&>) {
                return this->contains(x) && this->key_equal_(x, this->parents_.find(x)->second);
            }

        };  // class map_uf_base_

        /************************** IMPL FOR ARRAY-BASED UF *************************/

        template <
            typename T,
            bool TrackSize,
            typename ParentStorage,
            typename SizeStorage,
            typename KeyEqual = std::equal_to<>>
        class array_uf_base_ {
        public:
            using parent_storage = ParentStorage;
            using size_storage = adaptive_size_storage_t<T, TrackSize, SizeStorage>;
            using value_type = T;
            using size_type = typename parent_storage::size_type;
            using key_equal = KeyEqual;
        protected:
            parent_storage parents_;  // No default initialization
            [[no_unique_address]] size_storage sizes_;
            size_type set_count_{0};
            [[no_unique_address]] key_equal key_equal_{};

            constexpr const value_type& get_ensured_root(value_type x) const
            noexcept(std::is_nothrow_invocable_v<key_equal, const value_type&, const value_type&>) {
                while (!this->key_equal_(x, this->parents_[x])) {
                    x = this->parents_[x];
                }
                return this->parents_[x];
            }

            // Called only if the element x exists
            constexpr const value_type& get_ensured_root_and_compress(value_type x)
            noexcept(std::is_nothrow_invocable_v<key_equal, const value_type&, const value_type&>) {
                const value_type& root = get_ensured_root(x);
                // Path compression
                while (!this->key_equal_(x, this->parents_[x])) {
                    value_type tmp = this->parents_[x];
                    this->parents_[x] = root;
                    x = tmp;
                }
                return root;
            }

            constexpr void unite_existing_roots(value_type root_x, value_type root_y)
            noexcept(std::is_nothrow_invocable_v<key_equal, const value_type&, const value_type&>) {
                if (this->key_equal_(root_x, root_y)) [[unlikely]] {
                    return;
                }
                if constexpr (TrackSize) {
                    if (this->sizes_[root_x] > this->sizes_[root_y]) {
                        // If the set x belongs to is larger, merge the set of y to it
                        this->parents_[root_y] = root_x;
                        this->sizes_[root_x] += this->sizes_[root_y];
                    } else {
                        // Otherwise, merge the set of x to the set of y
                        this->parents_[root_x] = root_y;
                        this->sizes_[root_y] += this->sizes_[root_x];
                    }
                } else {
                    this->parents_[root_x] = root_y;
                }
                --this->set_count_;
            }

        public:

            constexpr array_uf_base_() = delete; // For array-based uf, there must be an initial size

            explicit constexpr array_uf_base_(const size_type n)
            requires std::default_initializable<key_equal>
            : array_uf_base_(n, KeyEqual{}) {   }

            constexpr array_uf_base_(const size_type n, key_equal eq)
            : parents_(n), set_count_{n}, key_equal_{ std::move(eq) } {
                std::iota(parents_.begin(), parents_.end(), 0);
                if constexpr (TrackSize) {
                    sizes_.assign(n, 1);
                }
            }

            constexpr array_uf_base_(const array_uf_base_&) = default;

            constexpr array_uf_base_(array_uf_base_&&)
            noexcept(
                    std::is_nothrow_move_constructible_v<parent_storage> &&
                    std::is_nothrow_move_constructible_v<size_storage> &&
                    std::is_nothrow_move_constructible_v<key_equal>
            ) = default;

            constexpr array_uf_base_& operator=(const array_uf_base_&) = default;

            constexpr array_uf_base_& operator=(array_uf_base_&&)
            noexcept(
                    std::is_nothrow_move_assignable_v<parent_storage> &&
                    std::is_nothrow_move_assignable_v<size_storage> &&
                    std::is_nothrow_move_assignable_v<key_equal>
            ) = default;

            constexpr ~array_uf_base_() = default;

            /********************** CORE METHODS **********************/

            constexpr void resize(const size_type n) requires (!TrackSize) {
                const auto old_size = this->parents_.size();
                // Union find does not support reducing elements
                if (n <= old_size) {
                    return;
                }
                this->parents_.resize(n);
                // parents_[i] = i
                std::iota(this->parents_.begin() + old_size, this->parents_.begin() + n, old_size);
                this->set_count_ += n - old_size;
            }

            constexpr void resize(const size_type n) requires TrackSize {
                const auto old_size = this->parents_.size();

                if (n <= old_size) {
                    return;
                }
                //Strong exception guarantee
                bool s_ok = false;
                try {
                    this->sizes_.resize(n); s_ok = true;
                    this->parents_.resize(n);
                } catch (...) {
                    if (s_ok) this->sizes_.resize(old_size);
                    throw;
                }
                // parents_[i] = i
                std::iota(this->parents_.begin() + old_size, this->parents_.begin() + n, old_size);
                // sizes_[i] = 1
                std::fill(this->sizes_.begin() + old_size, this->sizes_.begin() + n, 1);
                this->set_count_ += n - old_size;
            }

            [[nodiscard]] constexpr bool contains(value_type x) const noexcept {
                return x >= 0U && x < this->parents_.size();
            }

            [[nodiscard]] constexpr size_type set_size(value_type x) const noexcept
            requires TrackSize {
                if (!this->contains(x)) {
                    return 0U;
                }
                return this->sizes_[this->get_ensured_root(x)];
            }

            [[nodiscard]] constexpr bool is_root(const value_type& x) const
            noexcept(std::is_nothrow_invocable_v<key_equal, const value_type&, const value_type&>) {
                return this->contains(x) && this->key_equal_(x, this->parents_[x]);
            }

        };

        template <typename T, bool TrackSize, typename ParentStorage, typename SizeStorage, typename KeyEqual>
        using uf_base = std::conditional_t<concepts::unordered_map<ParentStorage>,
                            map_uf_base_<T, TrackSize, ParentStorage, SizeStorage, KeyEqual>,
                            array_uf_base_<T, TrackSize, ParentStorage, SizeStorage, KeyEqual>>;

    }

    template <
        concepts::object T,
        bool TrackSize = true,
        detail::valid_parent_storage<T> ParentStorage = detail::default_parent_storage_t<T>,
        detail::valid_size_storage<T, TrackSize, ParentStorage> SizeStorage =
                detail::default_size_storage_t<T, TrackSize, ParentStorage>,
        concepts::comparison_functor<T> KeyEqual = std::equal_to<>>
    class basic_union_find : public detail::uf_base<T, TrackSize, ParentStorage, SizeStorage, KeyEqual> {
        static_assert(std::is_copy_assignable_v<T>, "Copy assignment is required for path compression and union");

        using base_uf = detail::uf_base<T, TrackSize, ParentStorage, SizeStorage, KeyEqual>;
    public:
        using parent_storage = typename base_uf::parent_storage;
        using size_storage = typename base_uf::size_storage;
        using value_type = T;
        using size_type = typename base_uf::size_type;
        using difference_type = typename parent_storage::difference_type;
        using container_type = ParentStorage;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = T&;
        using const_reference = const T&;
        using iterator = typename ParentStorage::iterator;
        using const_iterator = typename ParentStorage::const_iterator;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;
        using key_equal = KeyEqual;

        /********************** CONSTRUCTORS **********************/

        constexpr basic_union_find() requires std::default_initializable<base_uf> = default;

        template <typename VTy>
        constexpr basic_union_find(std::initializer_list<VTy> il)
        requires std::constructible_from<base_uf, const VTy*, const VTy*>
        : base_uf(il.begin(), il.end()) {   }

        template <typename... Args>
        explicit constexpr basic_union_find(Args&& ...args)
        : base_uf{std::forward<Args>(args)...} {    }

        /********************** CORE METHODS **********************/

        // Insert if the element does not exist, then find its root with path compression.
        [[nodiscard]] constexpr const_reference find_or_insert(const_reference x)
        noexcept(noexcept(this->get_ensured_root_and_compress(x))) {
            if (!this->contains(x)) {
                if constexpr (concepts::unordered_map<parent_storage>) {
                    this->try_emplace(x);
                } else {
                    this->resize(x + 1);
                }
            }
            return this->get_ensured_root_and_compress(x);
        }

        // UB if the element does not exist
        [[nodiscard]] constexpr const_reference unchecked_find(const_reference x)
        noexcept(noexcept(this->get_ensured_root_and_compress(x))) {
            return this->get_ensured_root_and_compress(x);
        }

        [[nodiscard]] constexpr const_reference unchecked_find(const_reference x) const
        noexcept(noexcept(this->get_ensured_root(x))) {
            return this->get_ensured_root(x);
        }

        [[nodiscard]] constexpr bool same_set(const_reference x, const_reference y)
        noexcept(noexcept(this->get_ensured_root_and_compress(x)) &&
                 std::is_nothrow_invocable_v<key_equal, const_reference, const_reference>) {
            return this->contains(x) && this->contains(y) &&
                   this->key_equal_(this->get_ensured_root_and_compress(x),
                                   this->get_ensured_root_and_compress(y));
        }

        [[nodiscard]] constexpr bool same_set(const_reference x, const_reference y) const
        noexcept(noexcept(this->get_ensured_root(x)) &&
                 std::is_nothrow_invocable_v<key_equal, const_reference, const_reference>) {
            return this->contains(x) && this->contains(y) &&
                   this->key_equal_(this->get_ensured_root(x), this->get_ensured_root(y));
        }

        constexpr void unite_new(const_reference x, const_reference y) {
            this->unite_existing_roots(this->find_or_insert(x), this->find_or_insert(y));
        }

        constexpr bool try_unite(const_reference x, const_reference y)
        noexcept(noexcept(this->get_ensured_root_and_compress(x)) &&
                 noexcept(this->unite_existing_roots(x, y))) {
            if (!this->contains(x) || !this->contains(y)) {
                return false;
            }
            this->unite_existing_roots(this->get_ensured_root_and_compress(x),
                                       this->get_ensured_root_and_compress(y));
            return true;
        }

        // UB if the element does not exist
        constexpr void unchecked_unite(const_reference x, const_reference y)
        noexcept(noexcept(this->get_ensured_root_and_compress(x)) &&
                 noexcept(this->unite_existing_roots(x, y))) {
            this->unite_existing_roots(this->get_ensured_root_and_compress(x),
                                       this->get_ensured_root_and_compress(y));
        }

        [[nodiscard]] constexpr size_type set_count() const noexcept {
            return this->set_count_;
        }

        /********************** CAPACITY **********************/

        [[nodiscard]] constexpr bool empty() const noexcept {
            return this->parents_.empty();
        }

        [[nodiscard]] constexpr size_type size() const noexcept {
            return this->parents_.size();
        }

        [[nodiscard]] constexpr std::size_t ssize() const noexcept {
            return static_cast<std::size_t>(this->size());
        }

        [[nodiscard]] constexpr size_type max_size() const noexcept {
            return this->parents_.max_size();
        }

        [[nodiscard]] constexpr size_type capacity() const noexcept
        requires requires() { { this->parents_.capacity() } -> std::same_as<size_type>; } {
            return this->parents_.capacity();
        }

        constexpr void reserve(const size_type n)
        requires (concepts::reservable_container<parent_storage>) {
            this->parents_.reserve(n);
            if constexpr (TrackSize && concepts::reservable_container<size_storage>) {
                this->sizes_.reserve(n);
            }
        }

        /********************** ITERATORS **********************/
        // Non-const iterators are not provided
        [[nodiscard]] const_iterator begin() const noexcept {
            return this->parents_.begin();
        }

        [[nodiscard]] const_iterator end() const noexcept {
            return this->parents_.end();
        }

        [[nodiscard]] const_reverse_iterator rbegin() const noexcept {
            return std::reverse_iterator{ this->begin() };
        }

        [[nodiscard]] const_reverse_iterator rend() const noexcept {
            return std::reverse_iterator{ this->end() };
        }

        [[nodiscard]] const_iterator cbegin() const noexcept {
            return this->parents_.cbegin();
        }

        [[nodiscard]] const_iterator cend() const noexcept {
            return this->parents_.cend();
        }

        [[nodiscard]] const_reverse_iterator crbegin() const noexcept {
            return std::reverse_iterator{ this->cend() };
        }

        [[nodiscard]] const_reverse_iterator crend() const noexcept {
            return std::reverse_iterator{ this->cbegin() };
        }

        /********************** UTILITIES **********************/

        constexpr void clear() noexcept
        requires (std::is_empty_v<key_equal> || requires { this->key_equal_.clear(); }) {
            this->parents_.clear();
            if (TrackSize) {
                this->sizes_.clear();
            }
            if constexpr (!std::is_empty_v<key_equal>) {
                this->key_equal_.clear();
            }
            this->set_count_ = 0U;
        }

        constexpr void swap(basic_union_find& other)
        noexcept(std::is_nothrow_swappable_v<parent_storage> &&
                 std::is_nothrow_swappable_v<size_storage> &&
                 (std::is_empty_v<key_equal> || std::is_nothrow_swappable_v<key_equal>))
        requires (std::is_empty_v<key_equal> || std::swappable<key_equal>) {
            using std::swap;
            swap(this->parents_, other.parents_);
            if constexpr (TrackSize) {
                swap(this->sizes_, other.sizes_);
            }
            if constexpr (!std::is_empty_v<key_equal>) {
                swap(this->key_equal_, other.key_equal_);
            }
            swap(this->set_count_, other.set_count_);
        }

        [[nodiscard]]
        friend constexpr bool operator==(const basic_union_find& lhs, const basic_union_find& rhs) noexcept
        requires std::equality_comparable<parent_storage> {
            return lhs.set_count_ == rhs.set_count_ && lhs.parents_ == rhs.parents_;
        }

    };

    /********************** TYPE ALIAS **********************/

    // union_find - the most standard version
    using union_find = basic_union_find<uint32_t, true, std::vector<uint32_t>, std::vector<uint32_t>>;

    // small_union_find -- up to 65536 elements with size tracking
    using small_union_find = basic_union_find<uint16_t, true, std::vector<uint16_t>, std::vector<uint16_t>>;

    // micro_union_find -- up to 256 elements, no size tracking
    using micro_union_find = basic_union_find<uint8_t, false>;

    template <uint16_t N>
    using static_union_find = basic_union_find<uint16_t, true,
                                    inplace_vector<uint16_t, N>, inplace_vector<uint16_t, N>>;

    template <typename T>
    using map_union_find = basic_union_find<T, true, std::unordered_map<T, T>, std::unordered_map<T, size_t>>;


    /********************** CTAD GUIDEs **********************/

    // From unsigned integral T
    template <std::unsigned_integral T>
    basic_union_find(T n) -> basic_union_find<T>;

    // From unsigned integral T and a custom comparator
    template <std::unsigned_integral T, concepts::comparison_functor<T> Comp>
    basic_union_find(T n, Comp) ->
        basic_union_find<T,
                         true,  // Tracks size
                         detail::default_parent_storage_t<T>,
                         detail::default_size_storage_t<T, true, detail::default_parent_storage_t<T>>,
                         Comp>;

    // From an iterator pair
    template <typename Iter>
    basic_union_find(Iter, Iter) ->
        basic_union_find<std::iter_value_t<Iter>,
                         true,
                         std::unordered_map<std::iter_value_t<Iter>, std::iter_value_t<Iter>>,
                         std::unordered_map<std::iter_value_t<Iter>, size_t>>;

    // From an iterator pair and a custom comparator
    template <typename Iter, concepts::comparison_functor<std::iter_value_t<Iter>> Comp>
    basic_union_find(Iter, Iter, Comp) ->
        basic_union_find<std::iter_value_t<Iter>,
                         true,
                         std::unordered_map<std::iter_value_t<Iter>, std::iter_value_t<Iter>>,
                         std::unordered_map<std::iter_value_t<Iter>, size_t>,
                         Comp>;

    // From a range
    template <typename Rng>
    basic_union_find(Rng&&) ->
    basic_union_find<std::ranges::range_value_t<Rng>,
                         true,
                         std::unordered_map<std::ranges::range_value_t<Rng>, std::ranges::range_value_t<Rng>>,
                         std::unordered_map<std::ranges::range_value_t<Rng>, size_t>>;

    // From a range and a comparator
    template <typename Rng, typename Comp>
    basic_union_find(Rng&&, Comp) ->
        basic_union_find<std::ranges::range_value_t<Rng>,
                         true,
                         std::unordered_map<std::ranges::range_value_t<Rng>, std::ranges::range_value_t<Rng>>,
                         std::unordered_map<std::ranges::range_value_t<Rng>, size_t>,
                         Comp>;

}
#endif //UNION_FIND_H
