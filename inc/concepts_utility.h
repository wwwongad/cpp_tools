
#ifndef CONCEPTS_UTILITY_H
#define CONCEPTS_UTILITY_H
#include <concepts>
#include <memory>
#include <type_traits>
#include <utility>
#include <ranges>
#include <compare>
#include <iterator>

namespace urlicht::concepts {

	/************************* CONCEPTS FOR VALUE TYPES, RANGE AND ITERATORS *************************/

	// Checks if a type can be copy assigned by memcpy
	// Note that std::is_trivially_copyable_v being true does not imply memcpy copy-assignable
	// For example, the copy assignment operator may be deleted.
	template <typename T>
	concept bitwise_copy_assignable =
		std::is_trivially_copy_assignable_v<T> && std::is_trivially_destructible_v<T>;

	template <typename T>
	concept bitwise_move_assignable =
		std::is_trivially_move_assignable_v<T> && std::is_trivially_destructible_v<T>;

	template <typename Rng, typename T>
	concept compatible_range =
            std::ranges::input_range<Rng> &&
            std::constructible_from<T, std::ranges::range_reference_t<Rng>>;

    template <typename Iter, typename T>
	concept compatible_iterator =
            std::input_iterator<Iter> &&
            std::constructible_from<T, std::iter_reference_t<Iter>>;

	template <typename Iter>
	concept rvalue_iterator =
			std::input_iterator<Iter> &&
			std::is_rvalue_reference_v<std::iter_reference_t<Iter>>;

	template <typename Iter>
	concept lvalue_iterator =
			std::input_iterator<Iter> &&
			std::is_lvalue_reference_v<std::iter_reference_t<Iter>>;

	template <typename Rng>
	concept lvalue_range =
			std::ranges::input_range<Rng> &&
			std::is_lvalue_reference_v<std::ranges::range_reference_t<Rng>>;

	template <typename Rng>
	concept rvalue_range =
			std::ranges::input_range<Rng> &&
			std::is_rvalue_reference_v<std::ranges::range_reference_t<Rng>>;

	template <typename Sentinel, typename Iter>
	concept sentinel_or_iter =
		    std::input_iterator<Iter> &&
		    (std::same_as<Sentinel, Iter> || std::sentinel_for<Sentinel, Iter>);

	template <typename T>
	concept decays_to_ptr =
			std::is_pointer_v<std::decay_t<T>>;


	/************************* CONCEPTS FOR COMPARISONS *************************/

	template<typename Compare, typename T, typename U = T>
	concept comparison_functor =
    requires(Compare comp, const T& val, const U& val2) {
		{ comp(val, val2) } -> std::convertible_to<bool>;
	}
    || requires(Compare comp, const T& val, const U& val2) {
        { comp(val2, val) } -> std::convertible_to<bool>;
    };

	template <typename T, typename U = T>
	concept less_comparable =
		// Requires that the comparison function takes const lvalue reference only
		// To ensure that the instances are not modified
		requires(const std::remove_reference_t<T>& lhs, const std::remove_reference_t<U>& rhs) {
			{ lhs < rhs } -> std::convertible_to<bool>;
		}
	|| (concepts::decays_to_ptr<T> && concepts::decays_to_ptr<U>);

	template <typename T, typename U = T>
	concept greater_comparable =
		requires(const std::remove_reference_t<T>& lhs, const std::remove_reference_t<U>& rhs) {
			{ lhs > rhs } -> std::convertible_to<bool>;
		}
	|| (concepts::decays_to_ptr<T> && concepts::decays_to_ptr<U>);

	template <typename T, typename U = T>
	concept equality_comparable = // Requires T() == U() only
		requires(const std::remove_reference_t<T>& lhs, const std::remove_reference_t<U>& rhs) {
			{ lhs == rhs } -> std::convertible_to<bool>;
		}
	|| (concepts::decays_to_ptr<T> && concepts::decays_to_ptr<U>);

	template <typename T, typename U = T>
	concept inequality_comparable = // Requires T() != U() only
		requires(const std::remove_reference_t<T>& lhs, const std::remove_reference_t<U>& rhs) {
			{ lhs != rhs } -> std::convertible_to<bool>;
		}
	|| (concepts::decays_to_ptr<T> && concepts::decays_to_ptr<U>);


	template <typename T, typename U = T>
	concept nothrow_equality_comparable =
		concepts::equality_comparable<T, U> &&
        noexcept(std::declval<const T&>() == std::declval<const U&>());

	template <typename T, typename U = T>
	concept nothrow_inequality_comparable =
		concepts::inequality_comparable<T, U> &&
        noexcept(std::declval<const T&>() != std::declval<const U&>());

	template <typename T, typename U = T>
	concept nothrow_less_comparable =
		concepts::less_comparable<T, U> &&
        noexcept(std::declval<const T&>() < std::declval<const U&>());

	template <typename T, typename U = T>
	concept nothrow_greater_comparable =
		concepts::greater_comparable<T, U> &&
        noexcept(std::declval<const T&>() > std::declval<const U&>());

	template <typename T, typename U = T>
	concept nothrow_three_way_comparable =
		std::three_way_comparable_with<T, U> &&
        noexcept(std::declval<const T&>() <=> std::declval<const U&>());

	/************************* CONCEPTS FOR CONTAINERS *************************/
	// The following concepts are strictly STL-compliant

	template <typename Alloc>
	concept allocator = std::default_initializable<Alloc> && requires {
		typename Alloc::value_type;
		// Note the only type alias required from Alloc is value_type. std::allocator_traits generates
		// the remaining ones (e.g. pointer, size_type) accordingly.
	}
	&& requires (Alloc alloc,
			typename std::allocator_traits<Alloc>::pointer p,
			typename std::allocator_traits<Alloc>::size_type n) {
		{ std::allocator_traits<Alloc>::allocate(alloc, n) }
		-> std::same_as<typename std::allocator_traits<Alloc>::pointer>;
		{ std::allocator_traits<Alloc>::deallocate(alloc, p, n) }
		-> std::same_as<void>;
#if __cplusplus >= 202302L
		{ std::allocator_traits<Alloc>::allocate_at_least(alloc, n) }
		-> std::same_as<std::allocation_result<
				typename std::allocator_traits<Alloc>::pointer,
				typename std::allocator_traits<Alloc>::size_type>>;
#endif
		// std::allocator_traits provides fallback methods for the remaining static methods (e.g. max_size )
	};

    // Minimum requirement of a container
	// May not include views like std::span and std::string_view
	// Assumes that value_type is copy constructible and move constructible
    template <typename Cont>
    concept container =
        std::is_object_v<Cont> &&
        std::default_initializable<Cont> &&
        std::constructible_from<Cont, const Cont&> &&
        std::constructible_from<Cont, Cont&&> &&
        std::assignable_from<Cont&, const Cont&> &&
        std::assignable_from<Cont&, Cont&&> &&
    requires {
      	typename Cont::value_type;
        typename Cont::size_type;
        typename Cont::difference_type;
        typename Cont::reference;
        typename Cont::const_reference;
        typename Cont::pointer;
        typename Cont::const_pointer;
        typename Cont::iterator;
        typename Cont::const_iterator;
    }
    && requires(Cont& cont, Cont& cont2) {
        // Assignment
        { cont = cont2 } -> std::same_as<Cont&>;
        { cont = std::move(cont2) } -> std::same_as<Cont&>;
		// Iterators
        { cont.begin() } -> std::same_as<typename Cont::iterator>;
        { cont.end() } -> std::same_as<typename Cont::iterator>;
    	// Modifiers
	    { cont.swap(cont2) } -> std::same_as<void>;
    }
	&& requires (const Cont& cont) { // const methods
    	// Capacity
	    { cont.empty() } -> std::convertible_to<bool>;
	    { cont.max_size() } -> std::convertible_to<typename Cont::size_type>;
    	// Iterators
	    { cont.begin() } -> std::same_as<typename Cont::const_iterator>;
	    { cont.cbegin() } -> std::same_as<typename Cont::const_iterator>;
	    { cont.end() } -> std::same_as<typename Cont::const_iterator>;
	    { cont.cend() } -> std::same_as<typename Cont::const_iterator>;
    };

	// Singly-linked list, e.g. std::forward_list
	template <typename Cont>
	concept forward_list =
		container<Cont> &&
		std::forward_iterator<typename Cont::iterator> &&
		// If value_type is comparable, then forward_list is comparable
		(!concepts::equality_comparable<typename Cont::value_type> || concepts::equality_comparable<Cont>) &&
		(!std::three_way_comparable<typename Cont::value_type> || std::three_way_comparable<Cont>)
	&& requires {
		typename Cont::allocator_type;
	}
	&& requires(Cont& c,
				typename Cont::value_type val,
				typename Cont::size_type size,
				typename Cont::const_iterator cpos) {
		// Element access
		{ c.front() } -> std::same_as<typename Cont::reference>;
		{ std::as_const(c).front() } -> std::same_as<typename Cont::const_reference>;

		// Iterators
		{ c.before_begin() } -> std::same_as<typename Cont::iterator>;
		{ std::as_const(c).cbefore_begin() } -> std::same_as<typename Cont::const_iterator>;
		{ std::as_const(c).before_begin() } -> std::same_as<typename Cont::const_iterator>;

		// Modifiers
		{ c.emplace_after(cpos, val) } -> std::same_as<typename Cont::iterator>;
		{ c.emplace_after(cpos, std::move(val)) } -> std::same_as<typename Cont::iterator>;
		{ c.erase_after(cpos) } -> std::same_as<typename Cont::iterator>;
		{ c.erase_after(cpos, cpos) } -> std::same_as<typename Cont::iterator>;
		{ c.emplace_front(val) } -> std::same_as<typename Cont::reference>;  // Returns ref since c++17
		{ c.emplace_front(std::move(val)) } -> std::same_as<typename Cont::reference>;
		{ c.pop_front() } -> std::same_as<void>;
		{ c.resize(size, val) } -> std::same_as<void>;
		{ c.clear() } -> std::same_as<void>;
		// Optional: { c.insert_range_after(cpos, r) } -> std::same_as<void>;
		// Optional: { c.prepend_range(r) } -> std::same_as<void>;

		// Operations
		{ c.merge(c) } -> std::same_as<void>;
		{ c.merge(std::move(c)) } -> std::same_as<void>;
		{ c.splice_after(cpos, c) } -> std::same_as<void>;
		{ c.splice_after(cpos, std::move(c)) } -> std::same_as<void>;
		{ c.remove(val) } -> std::same_as<typename Cont::size_type>; // After C++20
		// Optional: { c.reverse() } -> std::same_as<void>;
		// Optional: { c.sort() } -> std::same_as<void>;
		// Optional: { c.unique() } -> std::same_as<typename Cont::size_type>;
	}
	&& (!std::default_initializable<typename Cont::value_type> ||
		requires (Cont& c, typename Cont::size_type size) { c.resize(size); });

	// Doubly-linked list, e.g. std::list
	template <typename Cont>
	concept list =
		container<Cont> &&
		std::bidirectional_iterator<typename Cont::iterator> &&
		std::bidirectional_iterator<typename Cont::const_iterator> &&
		(!concepts::equality_comparable<typename Cont::value_type> || concepts::equality_comparable<Cont>)
	&& requires {
		typename Cont::allocator_type;
		typename Cont::reverse_iterator;
		typename Cont::const_reverse_iterator;
	}
	&& requires(Cont& c,
				typename Cont::value_type value,
				typename Cont::size_type size,
				typename Cont::const_iterator cpos) {
		// Element access
		{ c.front() } -> std::same_as<typename Cont::reference>;
		{ std::as_const(c).front() } -> std::same_as<typename Cont::const_reference>;
		{ c.back() } -> std::same_as<typename Cont::reference>;
		{ std::as_const(c).back() } -> std::same_as<typename Cont::const_reference>;

		// Iterators
		{ c.rbegin() } -> std::same_as<typename Cont::reverse_iterator>;
		{ c.rend() } -> std::same_as<typename Cont::reverse_iterator>;
		{ c.crbegin() } -> std::same_as<typename Cont::const_reverse_iterator>;
		{ c.crend() } -> std::same_as<typename Cont::const_reverse_iterator>;
		{ std::as_const(c).rbegin() } -> std::same_as<typename Cont::const_reverse_iterator>;
		{ std::as_const(c).rend() } -> std::same_as<typename Cont::const_reverse_iterator>;

		// Capacity
		{ std::as_const(c).size() } -> std::same_as<typename Cont::size_type>;

		// Modifiers
		{ c.emplace(cpos, value) } -> std::same_as<typename Cont::iterator>;
		{ c.emplace(cpos, std::move(value)) } -> std::same_as<typename Cont::iterator>;
		{ c.erase(cpos) } -> std::same_as<typename Cont::iterator>;
		{ c.erase(cpos, cpos) } -> std::same_as<typename Cont::iterator>;
		{ c.emplace_front(value) } -> std::same_as<typename Cont::reference>;
		{ c.emplace_front(std::move(value)) } -> std::same_as<typename Cont::reference>;
		{ c.emplace_back(value) } -> std::same_as<typename Cont::reference>;
		{ c.emplace_back(std::move(value)) } -> std::same_as<typename Cont::reference>;
		{ c.pop_front() } -> std::same_as<void>;
		{ c.pop_back() } -> std::same_as<void>;
		{ c.resize(size, value) } -> std::same_as<void>;
		{ c.clear() } -> std::same_as<void>;
		// Optional: { c.append_range(c) } -> std::same_as<void>;
		// Optional: { c.insert_range(c) } -> std::convertible_to<typename Cont::iterator>;
		// Optional: { c.prepend_range(c) } -> std::same_as<void>;

		// Operations
		{ c.merge(c) } -> std::same_as<void>;
		{ c.merge(std::move(c)) } -> std::same_as<void>;
		{ c.splice(cpos, c) } -> std::same_as<void>;
		{ c.splice(cpos, std::move(c)) } -> std::same_as<void>;
		{ c.remove(value) } -> std::same_as<typename Cont::size_type>;
		// Optional: { c.reverse() } -> std::same_as<void>;
		// Optional: { c.sort() } -> std::same_as<void>;
		// Optional: { c.unique() } -> std::same_as<typename Cont::size_type>;
	}
	&& (!std::default_initializable<typename Cont::value_type> ||
		requires (Cont& c, typename Cont::size_type size) { c.resize(size); });


	// Random access container, e.g. std::deque, std::vector
	template <typename Cont>
	concept random_access_container =
	    container<Cont> &&
	    std::random_access_iterator<typename Cont::iterator> &&
	    std::random_access_iterator<typename Cont::const_iterator> &&
	    (!concepts::equality_comparable<typename Cont::value_type> || concepts::equality_comparable<Cont>) &&
	    (!std::three_way_comparable<typename Cont::value_type> || std::three_way_comparable<Cont>)
    && requires {
		// Some random access container may not have an allocator
        typename Cont::reverse_iterator;
        typename Cont::const_reverse_iterator;
    }
    && requires(Cont& c,
                typename Cont::value_type value,
                typename Cont::size_type size,
                typename Cont::difference_type idx,
                typename Cont::const_iterator cpos) {
        // Element access
        { c[idx] } -> std::same_as<typename Cont::reference>;
        { std::as_const(c)[idx] } -> std::same_as<typename Cont::const_reference>;
        { c.at(idx) } -> std::same_as<typename Cont::reference>;
        { std::as_const(c).at(idx) } -> std::same_as<typename Cont::const_reference>;
        { c.front() } -> std::same_as<typename Cont::reference>;
        { std::as_const(c).front() } -> std::same_as<typename Cont::const_reference>;
        { c.back() } -> std::same_as<typename Cont::reference>;
        { std::as_const(c).back() } -> std::same_as<typename Cont::const_reference>;

        // Iterators
        { c.rbegin() } -> std::same_as<typename Cont::reverse_iterator>;
        { c.rend() } -> std::same_as<typename Cont::reverse_iterator>;
        { std::as_const(c).rbegin() } -> std::same_as<typename Cont::const_reverse_iterator>;
        { std::as_const(c).rend() } -> std::same_as<typename Cont::const_reverse_iterator>;

        // Capacity
        { std::as_const(c).size() } -> std::same_as<typename Cont::size_type>;
        // Optional: { c.shrink_to_fit() } -> std::same_as<void>;

        // Modifiers
        { c.emplace(cpos, value) } -> std::same_as<typename Cont::iterator>;
        { c.emplace(cpos, std::move(value)) } -> std::same_as<typename Cont::iterator>;
        { c.erase(cpos) } -> std::same_as<typename Cont::iterator>;
        { c.erase(cpos, cpos) } -> std::same_as<typename Cont::iterator>;
        { c.emplace_back(value) } -> std::same_as<typename Cont::reference>;
        { c.emplace_back(std::move(value)) } -> std::same_as<typename Cont::reference>;
        { c.pop_back() } -> std::same_as<void>;
        { c.resize(size, value) } -> std::same_as<void>;
        { c.clear() } -> std::same_as<void>;
		// Optional: { c.append_range(c) } -> std::same_as<void>;
		// Optional: { c.insert_range(c) } -> std::convertible_to<typename Cont::iterator>;
    }
    && (!std::default_initializable<typename Cont::value_type> ||
        requires (Cont& c, typename Cont::size_type size) { c.resize(size); });


	// contiguous container, e.g. std::vector
	template <typename Cont>
	concept contiguous_container =
		random_access_container<Cont> &&
		std::contiguous_iterator<typename Cont::iterator> &&
		std::contiguous_iterator<typename Cont::const_iterator> &&
    requires(Cont& c, typename Cont::size_type size) {
        { c.data() } -> std::same_as<typename Cont::pointer>;
        { std::as_const(c).data() } -> std::same_as<typename Cont::const_pointer>;
        { c.capacity() } -> std::same_as<typename Cont::size_type>;
        { c.reserve(size) } -> std::same_as<void>;
    };

	// Double-ended queue
	template <typename Deq>
	concept deque =
		concepts::random_access_container<Deq>
	&& requires() {
		typename Deq::allocator_type;
	}
	&& requires(Deq& d, typename Deq::value_type value) {
		{ d.emplace_front(value) } -> std::same_as<typename Deq::value_type>;
		{ d.emplace_front(std::move(value)) } -> std::same_as<typename Deq::value_type>;
		// Optional: { d.prepend_range(d) } -> std::same_as<void>;
	};

	template<typename Container>
	concept reservable_container = container<Container> &&
		requires(Container cont, typename Container::size_type size) {
		{ cont.capacity() } -> std::convertible_to<typename Container::size_type>;
		{ cont.reserve(size) } -> std::same_as<void>;
		{ cont.shrink_to_fit() } -> std::same_as<void>;
	};

}


#endif //CONCEPTS_UTILITY_H
