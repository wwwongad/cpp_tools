
#ifndef BASE_HEAP__H
#define BASE_HEAP__H

#include <vector>
#include <concepts>
#include <concepts_uility.h>
#include <functional>
#include <ranges>

namespace urlicht::detail {

    template <
        typename T,
        concepts::random_access_container Container = std::vector<T>,
        concepts::compatible_functor<T> Compare = std::less<>>
    requires std::same_as<T, typename Container::value_type> && std::is_object_v<T>
    class base_heap {
    public:
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
    protected:
        Container data_;
        [[no_unique_address]] Compare compare_;

        //called after checking all the concepts requirements
        template <concepts::compatible_range<value_type> Rng>
        constexpr void append_range_noheapify(Rng&& range) {
            if constexpr (std::is_rvalue_reference_v<Rng>) {
                append_range_noheapify(
                        std::make_move_iterator(std::ranges::begin(range)),
                        std::make_move_iterator(std::ranges::end(range)));
            }
            append_range_noheapify(std::ranges::begin(range), std::ranges::end(range));
        }

        template <concepts::compatible_iterator<value_type> InputIt,
                  concepts::sentinel_or_iter<InputIt> Sentinel>
        constexpr void append_range_noheapify(InputIt begin, Sentinel end) {
            if constexpr (std::random_access_iterator<InputIt> &&
                          concepts::reservable_container<Container>) {
                const auto dist = static_cast<size_type>(end - begin);
                this->data_.reserve(dist + this->data_.size());
            }
            for (auto it = begin; it != end; ++it) {
                data_.emplace_back(std::forward<decltype(*it)>(*it));
            }
        }

        //Constructors
        //Default constructor
        constexpr base_heap()
        noexcept(std::is_nothrow_default_constructible_v<Container> &&
                 std::is_nothrow_default_constructible_v<Compare>)
        requires std::default_initializable<Container> && std::default_initializable<Compare>
            : data_{}, compare_{} {}

        //Constructor with Compare provided
        template <typename Cmp>
        requires std::constructible_from<Compare, Cmp&&> && std::default_initializable<Container>
        explicit(!std::convertible_to<Cmp&&, Compare>)
        constexpr base_heap(Cmp&& cmp)
            noexcept(std::is_nothrow_constructible_v<Compare, Cmp&&> &&
                     std::is_nothrow_default_constructible_v<Container>)
            : data_{}, compare_{std::forward<Cmp>(cmp)} {
        }

        /* Container constructors are covered by range constructors */

        //Iterator constructors
        template <std::input_iterator InputIt, concepts::sentinel_or_iter<InputIt> Sentinel>
        requires std::default_initializable<Compare> &&
                 std::constructible_from<Container, InputIt, Sentinel>
        explicit constexpr base_heap(InputIt begin, Sentinel end)
            : data_(begin, end), compare_{} {
        }

        template <std::input_iterator InputIt,
                  concepts::sentinel_or_iter<InputIt> Sentinel, typename Cmp>
        requires std::constructible_from<Compare, Cmp&&> &&
                 std::constructible_from<Container, InputIt, Sentinel>
        explicit constexpr base_heap(InputIt begin, Sentinel end, Cmp&& cmp)
            : data_(begin, end), compare_{std::forward<Cmp>(cmp)} {
        }

        template <concepts::compatible_iterator<value_type> InputIt,
                  concepts::sentinel_or_iter<InputIt> Sentinel>
        requires std::default_initializable<Compare> &&
                (!std::constructible_from<Container, InputIt, Sentinel>) &&
                 std::default_initializable<Container>
        explicit constexpr base_heap(InputIt begin, Sentinel end)
            : data_{}, compare_{} {
            append_range_noheapify(begin, end);
        }

        template <concepts::compatible_iterator<value_type> InputIt,
                  concepts::sentinel_or_iter<InputIt> Sentinel, typename Cmp>
        requires std::constructible_from<Compare, Cmp&&> &&
                (!std::constructible_from<Container, InputIt, Sentinel>) &&
                 std::default_initializable<Container>
        explicit constexpr base_heap(InputIt begin, Sentinel end, Cmp&& cmp)
            : data_{}, compare_{std::forward<Cmp>(cmp)} {
            append_range_noheapify(begin, end);
        }

        //Range Constructors
        template <std::ranges::input_range Rng>
        requires std::default_initializable<Compare> &&
                 std::constructible_from<Container, Rng&&>
        explicit constexpr base_heap(Rng&& range)
            : data_(std::forward<Rng>(range)), compare_{} {
        }

        template <std::ranges::input_range Rng, typename Cmp>
        requires std::constructible_from<Compare, Cmp&&> &&
                 std::constructible_from<Container, Rng&&>
        explicit constexpr base_heap(Rng&& range, Cmp&& cmp)
            : data_(std::forward<Rng>(range)), compare_{std::forward<Cmp>(cmp)} {
        }

        template <concepts::compatible_range<value_type> Rng>
        requires std::default_initializable<Compare> &&
                (!std::constructible_from<Container, Rng&&>) &&
                 std::default_initializable<Container>
        explicit constexpr base_heap(Rng&& range)
            : data_{}, compare_{} {
            append_range_noheapify(range);
        }

        template <concepts::compatible_range<value_type> Rng, typename Cmp>
        requires std::constructible_from<Compare, Cmp&&> &&
                (!std::constructible_from<Container, Rng&&>) &&
                 std::default_initializable<Container>
        explicit constexpr base_heap(Rng&& range, Cmp&& cmp)
            : data_{}, compare_{std::forward<Cmp>(cmp)} {
            append_range_noheapify(range);
        }

        constexpr ~base_heap() = default;

    public:
        /* The following methods are supposed to be universal among all derived classes. */
        //accessors
        [[nodiscard]] constexpr const_reference top() const noexcept {
            return data_[0];
        }

        [[nodiscard]] constexpr const_reference back() const noexcept(noexcept(data_.back())) {
            return data_.back();
        }

        [[nodiscard]] constexpr const value_compare& get_value_compare() const noexcept {
            return compare_;
        }

        [[nodiscard]] constexpr const container_type& get_container() const noexcept {
            return data_;
        }

        //Modifier
        constexpr void pop_back() noexcept(noexcept(data_.pop_back())) {
            data_.pop_back();
        }

        // Iterators
        // Note that iteration over a heap is in general meaningless
        [[nodiscard]] constexpr const_iterator begin() const noexcept {
            return data_.begin();
        }

        [[nodiscard]] constexpr const_iterator end() const noexcept {
            return data_.end();
        }

        [[nodiscard]] constexpr const_reverse_iterator rbegin() const noexcept {
            return std::reverse_iterator{ begin() };
        }

        [[nodiscard]] constexpr const_reverse_iterator rend() const noexcept {
            return std::reverse_iterator{ end() };
        }

        [[nodiscard]] constexpr const_iterator cbegin() const noexcept {
            return data_.cbegin();
        }

        [[nodiscard]] constexpr const_iterator cend() const noexcept {
            return data_.cend();
        }

        [[nodiscard]] constexpr const_reverse_iterator crbegin() const noexcept {
            return std::reverse_iterator{ cend() };
        }

        [[nodiscard]] constexpr const_reverse_iterator crend() const noexcept {
            return std::reverse_iterator{ cbegin() };
        }

        //Capacity
        [[nodiscard]] constexpr bool empty() const noexcept(noexcept(data_.empty())) {
            return data_.empty();
        }

        [[nodiscard]] constexpr size_type size() const noexcept(noexcept(data_.size())) {
            return data_.size();
        }

        [[nodiscard]] constexpr size_type max_size() const noexcept(noexcept(data_.max_size())) {
            return data_.max_size();
        }

        [[nodiscard]] constexpr size_type capacity() const noexcept(noexcept(data_.capacity()))
        requires requires { data_.capacity(); } {
            return data_.capacity();
        }

        constexpr void shrink_to_fit() noexcept(noexcept(data_.shrink_to_fit()))
        requires concepts::reservable_container<Container> {
            data_.shrink_to_fit();
        }

        constexpr void reserve(size_type i)
        requires concepts::reservable_container<Container> {
            data_.reserve(i);
        }

    }; // base_heap_
    }

#endif //BASE_HEAP__H
