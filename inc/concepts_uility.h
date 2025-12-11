
#ifndef CONCEPTS_UILITY_H
#define CONCEPTS_UILITY_H
#include <concepts>
#include <memory>
#include <type_traits>

	namespace urlicht::concepts {

	template <typename T>
	concept decays_to_ptr =
			std::is_pointer_v<std::decay_t<T>>;

	template <typename Rng, typename T>
	concept compatible_range =
            std::ranges::input_range<Rng> &&
            std::constructible_from<T, std::ranges::range_reference_t<Rng>>;

    template <typename Iter, typename T>
	concept compatible_iterator =
            std::forward_iterator<Iter> &&
            std::constructible_from<T, std::iter_reference_t<Iter>>;

	template <typename Iter>
	concept rvalue_iterator =
			std::forward_iterator<Iter> &&
			std::is_rvalue_reference_v<std::remove_reference_t<Iter>>;

	template <typename Iter>
	concept lvalue_iterator =
			std::forward_iterator<Iter> &&
			std::is_rvalue_reference_v<std::remove_reference_t<Iter>>;

	template <typename Sentinel, typename Iter>
	concept sentinel_or_iter =
		    std::input_iterator<Iter> &&
		    (std::same_as<Sentinel, Iter> || std::sentinel_for<Sentinel, Iter>);


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

	// Checks if a type can be copy assigned by memcpy
	// Note that std::is_trivially_copyable_v being true does not imply memcpy copy-assignable
	// For example, the copy assignment operator may be deleted.
	template <typename T>
	concept bitwise_copy_assignable =
		std::is_trivially_copy_assignable_v<T> && std::is_trivially_destructible_v<T>;

	template <typename T>
	concept bitwise_move_assignable =
		std::is_trivially_move_assignable_v<T> && std::is_trivially_destructible_v<T>;


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
    template <typename Cont>
    concept container =
        std::same_as<Cont, std::remove_reference_t<Cont>> &&
        std::default_initializable<Cont> &&
        std::constructible_from<Cont, const Cont&> &&
        std::constructible_from<Cont, Cont&&> &&
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
    && requires(const Cont& cont, Cont& cont2) {
        // Assignment
        { cont = cont2 } -> std::same_as<Cont&>;
        { cont = std::move(cont2) } -> std::same_as<Cont&>;
        // Capacity
    	{ cont.empty() } -> std::convertible_to<bool>;
        { cont.max_size() } -> std::convertible_to<typename Cont::size_type>;
		// Iterators
        { cont.begin() } -> std::convertible_to<typename Cont::const_iterator>;
        { cont.cbegin() } -> std::same_as<typename Cont::const_iterator>;
        { cont.end() } -> std::convertible_to<typename Cont::const_iterator>;
        { cont.cend() } -> std::same_as<typename Cont::const_iterator>;
    };

	template<typename Container>
	concept reservable_container = container<Container> &&
		requires(Container cont, typename Container::size_type size) {
		{ cont.capacity() } -> std::convertible_to<typename Container::size_type>;
		{ cont.reserve(size) } -> std::same_as<void>;
		{ cont.shrink_to_fit() } -> std::same_as<void>;
	};

	template <typename Rng, typename Container>
	concept constructible_range =
		std::ranges::input_range<Rng> &&
		concepts::container<Container> &&
		std::constructible_from<Container, Rng>;

}


#endif //CONCEPTS_UILITY_H
