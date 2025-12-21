
#ifndef INPLACE_VECTOR_H
#define INPLACE_VECTOR_H
#pragma once
#include <iterator>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include <concepts>
#include <type_traits>

#include "../concepts_utility.h"

namespace urlicht {

    namespace detail { // Implementation details, not expected to be used directly

    // Complying to the standard
    template <std::size_t N>
    using adaptive_size_type =
        std::conditional_t<N <= std::numeric_limits<uint8_t>::max(), uint8_t,
            std::conditional_t<N <= std::numeric_limits<uint16_t>::max(), uint16_t,
                std::conditional_t<N <= std::numeric_limits<uint32_t>::max(), uint32_t,
                    std::conditional_t<N <= std::numeric_limits<uint64_t>::max(), uint64_t, std::size_t>
                >
            >
        >;

    template <std::ranges::range Rng, std::integral size_type = std::size_t>
    size_type range_size(const Rng& rng){
        using namespace std::ranges;
        if constexpr (sized_range<Rng>){
            return static_cast<size_type>(size(rng));
        } else {
            return static_cast<size_type>(distance(rng));
        }
    }

    template<typename T, std::size_t N = 0>
    struct zero_sized_storage {
        static_assert(N == 0, "The size of zero_sized_storage must be zero");

        using size_type = uint8_t;
        using storage_type = std::byte;

        constexpr zero_sized_storage() noexcept = default;

        constexpr zero_sized_storage(const zero_sized_storage&) = default;

        constexpr zero_sized_storage(zero_sized_storage&&) = default;

        constexpr zero_sized_storage& operator=(const zero_sized_storage&) = default;

        constexpr zero_sized_storage& operator=(zero_sized_storage&&) = default;

        constexpr ~zero_sized_storage() = default;

        // For compatibility
        constexpr explicit zero_sized_storage(const size_type size) {
            if (size > 0) {
              throw std::bad_alloc();
            }
        }

        constexpr zero_sized_storage(const size_type size, const T& value) {
            if (size > 0) {
                throw std::bad_alloc();
            }
        }

        template <concepts::compatible_iterator<T> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr zero_sized_storage(Iter first, Sentinel last) {
            if (std::distance(first, last) > 0) {
                throw std::bad_alloc();
            }
        }

        template <concepts::compatible_range<T> Rng>
        constexpr explicit zero_sized_storage(Rng&& rng) {
            if (range_size(rng) > 0) {
                throw std::bad_alloc();
            }
        }

        template <typename U>
        constexpr zero_sized_storage(std::initializer_list<U> il) {
            if (il.size() > 0) {
                throw std::bad_alloc();
            }
        }

        template <std::input_iterator Iter>
        constexpr void unchecked_append_range(Iter first, size_type size) noexcept {
            if (size > 0) {
                std::terminate();
            }
        }

        template <std::input_iterator Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr void unchecked_append_range(Iter first, Sentinel last) noexcept {
            unchecked_append_range(first, std::distance(first, last));
        }

        template <concepts::compatible_range<T> Rng>
        constexpr void unchecked_append_range(Rng&& rng) noexcept {
            unchecked_append_range(std::ranges::begin(rng), range_size(rng));
        }

        constexpr void unchecked_append(size_type n, const T& value) noexcept {
            if (n > 0) {
                std::terminate();
            }
        }

        [[nodiscard]] static constexpr T* data() noexcept {
            return nullptr;
        }

        [[nodiscard]] static constexpr size_type size() noexcept {
            return 0U;
        }

        static constexpr void unchecked_set_size(const std::size_t size = 0) noexcept {
            if (size > 0) {
                std::terminate();
            }
        }
    };

    // storage for trivial types, using std::array as the underlying buffer
    template<typename T, std::size_t N>
    struct trivial_storage {
        static_assert(N > 0, "Use zero_sized_storage for N == 0.");
        static_assert(std::is_trivial_v<T>, "The storage type of trivial_storage must be trivial.");

        using size_type = adaptive_size_type<N>;
        using storage_type = std::conditional_t<std::is_const_v<T>, std::array<std::remove_const_t<T>, N>,
            std::array<T, N>>;
    private:
        alignas(alignof(T)) storage_type storage_;  // Not initialized
        size_type size_{0U};
    public:
        constexpr trivial_storage() = default;

        constexpr trivial_storage(const trivial_storage&) = default;

        constexpr trivial_storage(trivial_storage&&) = default;

        constexpr trivial_storage& operator=(const trivial_storage&) = default;

        constexpr trivial_storage& operator=(trivial_storage&&) = default;

        constexpr explicit trivial_storage(const size_type size)
        : size_{size} {
            if (size > N) [[unlikely]] {
              throw std::bad_alloc();
            }
            std::memset(storage_.data(), 0, sizeof(T) * size_);
        }

        constexpr explicit trivial_storage(const size_type size, const T& value) {
            if (size > N) [[unlikely]]  {
                throw std::bad_alloc();
            }
            unchecked_append(size, value);
        }

        // Iterator constructor
        template <concepts::compatible_iterator<T> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr trivial_storage(Iter first, Sentinel last)
        requires std::convertible_to<std::iter_reference_t<Iter>, T> // Implicit conversion for trivial types
        {
            const size_type size { std::distance(first, last) };
            if (size > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            unchecked_append_range(first, size);
        }

        // Range constructor
        template <concepts::compatible_range<T> Rng>
        constexpr explicit trivial_storage(Rng&& rng) {
            size_type size { range_size(rng) };
            if (size > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            unchecked_append_range(std::ranges::begin(rng), size);
        }

        // Initializer list constructor
        template <typename VTy>
        requires std::convertible_to<VTy, T>
        constexpr trivial_storage(std::initializer_list<VTy> list)
        : trivial_storage(list.begin(), list.end()) { }

        template<concepts::compatible_iterator<T> Iter>
        constexpr void unchecked_append_range(Iter first, const size_type size) noexcept // UB if out-of-bound
        {
            if constexpr (std::contiguous_iterator<Iter>) {
                std::memcpy(storage_.data() + size_, std::to_address(first), size); // constexpr since c++23
            } else {
                std::copy_n(first, size, storage_.data() + size_);
            }
            size_ += size;
        }

        template <concepts::compatible_iterator<T> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr trivial_storage(Iter first, Sentinel last) noexcept {
            unchecked_append_range(first, std::distance(first, last));
        }
        
        template <concepts::compatible_range<T> Rng>
        constexpr void unchecked_append_range(Rng&& rng) noexcept {
            size_type size { range_size(rng) };
            unchecked_append_range(std::ranges::begin(rng), size);
        }

        constexpr void unchecked_append(size_type size, const T& value) noexcept {
            if constexpr (sizeof(T) == 1) {
                std::memset(storage_.data() + size_, value, sizeof(T) * size);
            } else {
                std::fill(storage_.data() + size_, storage_.data() + size_ + size, value);
            }
            size_ += size;
        }

        constexpr ~trivial_storage() noexcept = default;

        [[nodiscard]] constexpr T* data() noexcept
        requires(!std::is_const_v<T>) {
            return storage_.data();
        }

        [[nodiscard]] constexpr const T* data() const noexcept {
            return storage_.data();
        }

        [[nodiscard]] constexpr size_type size() const noexcept {
            return size_;
        }

        constexpr void unchecked_set_size(size_type new_size) noexcept { // Undefined behaviour if new_size > N
            size_ = new_size;
        }

        constexpr void clear() noexcept {
            size_ = 0U;
        }
    };

    // Storage for non-trivial types using raw bytes, optimized for potentially trivial constructor/assignment operator
    template <typename T, std::size_t N>
    struct non_trivial_storage {
        static_assert(!std::is_trivial_v<T>, "Use trivial_storage for trivial value types.");
        static_assert(N > 0, "Use zero_sized_storage for N == 0.");

        using size_type = adaptive_size_type<N>;
        using storage_type = std::byte[N * sizeof(T)];

    private:
        alignas(alignof(T)) storage_type storage_;  // Not initialized
        size_type size_{0U};

    public:
        constexpr non_trivial_storage() = default;

        // Copy constructor
        constexpr non_trivial_storage(const non_trivial_storage& other)
        noexcept(std::is_nothrow_copy_constructible_v<T>)
        requires std::copy_constructible<T> {
            unchecked_append_range(other.data(), other.size());
        }

        // Move constructor
        constexpr non_trivial_storage(non_trivial_storage&& other)
        noexcept (std::is_nothrow_move_constructible_v<T>)
        requires std::move_constructible<T> {
            unchecked_append_range(std::make_move_iterator(other.data()), other.size());
            other.clear();
        }

        // Size constructor
        constexpr explicit non_trivial_storage(const size_type new_size)
        requires std::default_initializable<T> {
            if (new_size > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            size_ = new_size;
            if constexpr (!std::is_trivially_default_constructible_v<T>) {
                std::uninitialized_default_construct_n(this->data(), size_);
            }
        }

        // Valued size constructor
        constexpr non_trivial_storage(const size_type new_size, const T& value) {
            if (new_size > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            unchecked_append(new_size, value);
        }

        // Iterator constructor
        template <concepts::compatible_iterator<T> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr non_trivial_storage(Iter first, Sentinel last) {
            const auto m = std::distance(first, last);
            if (m > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            unchecked_append_range(first, m);
        }

        // Range constructor
        template <concepts::compatible_range<T> Rng>
        constexpr explicit non_trivial_storage(Rng&& rng) {
            const auto m { range_size(rng) };
            if (m > N) [[unlikely]] {
                throw std::bad_alloc();
            }
            unchecked_append_range(std::ranges::begin(rng), m);
        }

        // Initializer list constructor
        template <typename VTy>
        requires std::constructible_from<T, VTy>
        constexpr non_trivial_storage(std::initializer_list<VTy> list)
        : non_trivial_storage(list.begin(), list.end()) { }

        // Trivial copy assignment operator
        constexpr non_trivial_storage& operator=(const non_trivial_storage& other) noexcept
        requires concepts::bitwise_copy_assignable<T> {
            // Does not guard against self assignment since memmove is cheap
            std::memmove(this->data(), other.data(), other.size() * sizeof(T));
            size_ = other.size();
            return *this;
        }

        // Trivial move assignment operator
        constexpr non_trivial_storage& operator=(non_trivial_storage&& other) noexcept
        requires concepts::bitwise_move_assignable<T> {
            this->operator=(other); // Delegates to copy assignment operator
            other.unchecked_set_size(0U);
            return *this;
        }

        // Non-trivial copy assignment operator
        constexpr non_trivial_storage& operator=(const non_trivial_storage& other)
        noexcept(std::is_nothrow_copy_assignable_v<T>
                && std::is_nothrow_copy_constructible_v<T>
                && std::is_nothrow_destructible_v<T>)
        requires std::is_copy_assignable_v<T> && (!concepts::bitwise_copy_assignable<T>) {
            if (this == &other) [[unlikely]] {
                return *this;
            }
            const size_type k = other.size() > size_ ? size_ : other.size();
            if (other.size() > size_) {
                std::uninitialized_copy_n(other.data() + k,other.size() - k, this->data() + k);
            } else {
                std::destroy_n(this->data() + k, size_ - k);
            }
            std::copy_n(other.data(), k, this->data());

            size_ = other.size();
            return *this;
        }

        // Non-trivial move assignment operator
        constexpr non_trivial_storage& operator=(non_trivial_storage&& other)
        noexcept(std::is_nothrow_move_assignable_v<T>
                && std::is_nothrow_move_constructible_v<T>
                && std::is_nothrow_destructible_v<T>)
        requires std::is_move_assignable_v<T> && (!concepts::bitwise_move_assignable<T>) {
            if (this == &other) [[unlikely]] {
                return *this;
            }
            const size_type k = other.size() > size_ ? size_ : other.size();
            if (other.size() > size_) {
                std::uninitialized_move_n(other.data() + k,other.size() - k, this->data() + k);
            } else {
                std::destroy_n(this->data() + k, size_ - k);
            }
            std::move(other.data(), other.data() + k, this->data());

            size_ = other.size();
            other.clear();
            return *this;
        }

        // Append range - UB if out-of-bound
        template <concepts::compatible_iterator<T> Iter>
        constexpr void unchecked_append_range(Iter first, std::size_t m)
        noexcept(std::is_trivially_copyable_v<T>
                || (concepts::rvalue_iterator<Iter> && std::is_nothrow_move_constructible_v<T>)
                || (concepts::lvalue_iterator<Iter> && std::is_nothrow_copy_constructible_v<T>))
        requires (concepts::lvalue_iterator<Iter> && std::copy_constructible<T>)
            || (concepts::rvalue_iterator<Iter> && (std::copy_constructible<T> || std::move_constructible<T>)) {
            if constexpr (std::is_trivially_copyable_v<T> && std::contiguous_iterator<Iter>) {
                std::memcpy(this->data() + size_, std::to_address(first), m * sizeof(T));
            } else if constexpr (concepts::rvalue_iterator<Iter>) {
                std::uninitialized_move_n(first, m, this->data() + size_);
            } else {
                std::uninitialized_copy_n(first, m, this->data() + size_);
            }
            size_ += m;
        }

        template <concepts::compatible_iterator<T> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr void unchecked_append_range(Iter first, Sentinel last)
        noexcept(noexcept(unchecked_append_range(first, std::distance(first, last))))
        requires requires () { unchecked_append_range(first, std::distance(first, last)); } {
            unchecked_append_range(first, std::distance(first, last));
        }

        template <concepts::compatible_range<T> Rng>
        constexpr void unchecked_append_range(Rng&& rng)
        noexcept(std::is_trivially_copyable_v<T>
                || (concepts::rvalue_range<Rng&&> && std::is_nothrow_move_constructible_v<T>)
                || (concepts::lvalue_range<Rng&&> && std::is_nothrow_copy_constructible_v<T>))
        requires (concepts::lvalue_range<Rng&&> && std::copy_constructible<T>)
            || (concepts::rvalue_range<Rng&&> && (std::move_constructible<T> || std::copy_constructible<T>))  {
            if constexpr (std::is_rvalue_reference_v<Rng&&>) {
                unchecked_append_range(std::make_move_iterator(std::ranges::begin(rng)),
                    range_size(rng));
            } else {
                unchecked_append_range(std::ranges::begin(rng),
                    range_size(rng));
            }
        }

        constexpr void unchecked_append(size_type n, const T& value)
        noexcept(std::is_nothrow_copy_constructible_v<T>) {
            if constexpr (sizeof(T) == 1 && std::is_trivially_copy_constructible_v<T>) {
                std::memset(this->data() + size_, value, n);
            } else {
                std::uninitialized_fill_n(this->data() + size_, n, value);
            }
            size_ += n;
        }

        // Destructor
        constexpr ~non_trivial_storage() noexcept(std::is_nothrow_destructible_v<T>) {
            if constexpr (!std::is_trivially_destructible_v<T>) {
                std::destroy_n(this->data(), size_);
            }
        }

        [[nodiscard]] constexpr T* data() noexcept
        requires (!std::is_const_v<T>) {
            return reinterpret_cast<T*>(storage_);
        }

        [[nodiscard]] constexpr const T* data() const noexcept {
            return reinterpret_cast<const T*>(storage_);
        }

        [[nodiscard]] constexpr size_type size() const noexcept {
            return size_;
        }

        constexpr void unchecked_set_size(std::size_t new_size) noexcept {
            size_ = static_cast<size_type>(new_size);
        }

        constexpr void clear() noexcept(std::is_nothrow_destructible_v<T>) {
            if constexpr (!std::is_trivially_destructible_v<T>) {
                std::destroy_n(this->data(), size_);
            }
            size_ = 0U;
        }

    };

    template <typename T, size_t N>
    using adaptive_storage_type = std::conditional_t<N == 0, zero_sized_storage<T>,
        std::conditional_t<std::is_trivial_v<T>, trivial_storage<T, N>, non_trivial_storage<T, N>>>;

    } // namespace detail

    template <typename T, size_t N>
    requires std::is_object_v<T>
    class inplace_vector {
    public:
        using value_type = T;
        using storage_type = detail::adaptive_storage_type<T, N>;
        using size_type = typename storage_type::size_type;
        using difference_type = std::ptrdiff_t;
        using reference = T&;
        using const_reference = const T&;
        using pointer = T*;
        using const_pointer = const T*;
        using iterator = pointer;
        using const_iterator = const_pointer;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    private:
        [[no_unique_address]] storage_type storage_;

    public:
        /****************************** CONSTRUCTORS ******************************/

        // Default constructor -- initializes size only
        constexpr inplace_vector() noexcept = default;

        // Copy constructor
        constexpr inplace_vector(const inplace_vector& other)
        noexcept(std::is_nothrow_copy_constructible_v<storage_type>)
        requires std::copy_constructible<storage_type> = default;

        // Move constructor
        constexpr inplace_vector(inplace_vector&& other)
        noexcept(std::is_nothrow_move_constructible_v<storage_type>)
        requires std::move_constructible<storage_type> = default;

        // Size constructor - default initializes the first {size} elements
        constexpr explicit inplace_vector(const size_type size)
        requires std::constructible_from<storage_type, size_type>
        : storage_(size) { }

        // Valued size constructor - fill the first {size} elements with {value}
        constexpr inplace_vector(const size_type size, const T& value)
        requires std::constructible_from<storage_type, size_type, const T&>
        : storage_(size, value) { }

        // Iterator constructor
        template <std::forward_iterator Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr inplace_vector(Iter first, Sentinel last)
        requires std::constructible_from<storage_type, Iter, Sentinel>
        : storage_(first, last) { }

        // Range constructor
        template <concepts::compatible_range<T> Rng>
        constexpr explicit inplace_vector(Rng&& rng)
        requires std::constructible_from<storage_type, Rng>
        : storage_(std::forward<Rng>(rng)) { }

        // Initializer list constructor
        template <typename VTy>
        constexpr inplace_vector(std::initializer_list<VTy> init)
        requires std::constructible_from<storage_type, std::initializer_list<VTy>>
        : storage_(init) { }

        // Destructor
        constexpr ~inplace_vector()
        noexcept(std::is_nothrow_destructible_v<storage_type>) = default;

        // Assignment
        // Copy assignment operator
        constexpr inplace_vector& operator=(const inplace_vector& other)
        noexcept(std::is_nothrow_copy_assignable_v<storage_type>)
        requires std::assignable_from<storage_type&, const storage_type&> = default;

        // Move assignment operator
        constexpr inplace_vector& operator=(inplace_vector&& other)
        noexcept(std::is_nothrow_move_assignable_v<storage_type>)
        requires std::assignable_from<storage_type&, storage_type&&> = default;

        /****************************** ELEMENT ACCESS ******************************/

        [[nodiscard]] constexpr pointer data() noexcept
        requires (!std::is_const_v<value_type>) {
            return storage_.data();
        }

        [[nodiscard]] constexpr const_pointer data() const noexcept {
            return storage_.data();
        }

        [[nodiscard]] constexpr reference front() noexcept
        requires (!std::is_const_v<value_type>) {
            return storage_.data()[0];
        }

        [[nodiscard]] constexpr const_reference front() const noexcept {
            return storage_.data()[0];
        }

        [[nodiscard]] constexpr reference back() noexcept
        requires (!std::is_const_v<value_type>) {
            return storage_.data()[storage_.size() - 1];
        }

        [[nodiscard]] constexpr const_reference back() const noexcept {
            return storage_.data()[storage_.size() - 1];
        }

        [[nodiscard]] constexpr reference operator[](const size_type index) noexcept
        requires (!std::is_const_v<value_type>) {
            return storage_.data()[index];
        }

        [[nodiscard]] constexpr const_reference operator[](const size_type index) const noexcept {
            return storage_.data()[index];
        }

        [[nodiscard]] constexpr reference at(const size_type index)
        requires (!std::is_const_v<value_type>) {
            if (index >= storage_.size()) [[unlikely]] {
                throw std::out_of_range("inplace_vector::at");
            }
            return storage_.data()[index];
        }

        [[nodiscard]] constexpr const_reference at(const size_type index) const {
            if (index >= storage_.size()) [[unlikely]] {
                throw std::out_of_range("inplace_vector::at");
            }
            return storage_.data()[index];
        }

        /****************************** CAPACITY ******************************/

        [[nodiscard]] constexpr size_type size() const noexcept {
            return storage_.size();
        }

        [[nodiscard]] constexpr std::size_t ssize() const noexcept {
            return static_cast<std::size_t>(this->size());
        }

        [[nodiscard]] constexpr bool empty() const noexcept {
            return this->size() == 0U;
        }

        [[nodiscard]] static consteval size_type capacity() noexcept {
            return N;
        }

        [[nodiscard]] static consteval size_type max_size() noexcept {
            return N;
        }

        constexpr void reserve(size_type size) {
            if (size > N) [[unlikely]] {
                throw std::bad_alloc{};
            }
        }

        static consteval void shrink_to_fit() noexcept { /* nop */ }

        /****************************** ITERATORS ******************************/

        [[nodiscard]] constexpr iterator begin() noexcept
        requires (!std::is_const_v<value_type>) {
            return storage_.data();
        }

        [[nodiscard]] constexpr const_iterator begin() const noexcept {
            return storage_.data();
        }

        [[nodiscard]] constexpr iterator end() noexcept
        requires (!std::is_const_v<value_type>) {
            return begin() + storage_.size();
        }

        [[nodiscard]] constexpr const_iterator end() const noexcept {
            return begin() + storage_.size();
        }

        [[nodiscard]] constexpr reverse_iterator rbegin() noexcept
        requires (!std::is_const_v<value_type>) {
            return std::reverse_iterator{ end() };
        }

        [[nodiscard]] constexpr reverse_iterator rend() noexcept
        requires (!std::is_const_v<value_type>) {
            return std::reverse_iterator{ begin() };
        }

        [[nodiscard]] constexpr const_reverse_iterator rbegin() const noexcept {
            return std::reverse_iterator{ end() };
        }

        [[nodiscard]] constexpr const_reverse_iterator rend() const noexcept {
            return std::reverse_iterator{ begin() };
        }

        [[nodiscard]] constexpr const_iterator cbegin() const noexcept {
            return storage_.data();
        }

        [[nodiscard]] constexpr const_iterator cend() const noexcept {
            return begin() + storage_.size();
        }

        [[nodiscard]] constexpr const_reverse_iterator crbegin() const noexcept {
            return std::reverse_iterator{ cend() };
        }

        [[nodiscard]] constexpr const_reverse_iterator crend() const noexcept {
            return std::reverse_iterator{ cbegin() };
        }

        /****************************** MODIFIERS ******************************/

        template <typename ...Args>
        requires std::constructible_from<value_type, Args&&...>
        constexpr reference unchecked_emplace_back(Args&&... args)
        noexcept(std::is_nothrow_constructible_v<value_type, Args&&...>) {
            // UB if out-of-range
            std::construct_at(this->end(), std::forward<Args>(args)...);
            storage_.unchecked_set_size(this->size() + 1U);
            return this->back();
        }

        template <typename ...Args>
        requires std::constructible_from<value_type, Args&&...>
        constexpr pointer try_emplace_back(Args&&... args)
        noexcept(std::is_nothrow_constructible_v<value_type, Args&&...>) {
            if (this->size() == N) [[unlikely]] {
                return nullptr;
            }
            return &this->unchecked_emplace_back(std::forward<Args>(args)...);
        }

        template <typename ...Args>
        requires std::constructible_from<value_type, Args&&...>
        constexpr reference emplace_back(Args&&... args) {
            if (this->size() == N) [[unlikely]] {
                throw std::bad_alloc{};
            }
            return this->unchecked_emplace_back(std::forward<Args>(args)...);
        }

        constexpr void unchecked_pop_back()
        noexcept(std::is_nothrow_destructible_v<value_type>) {
            // UB if empty
            if constexpr (!std::is_trivially_destructible_v<value_type>) {
                std::destroy_at(this->end());
            }
            storage_.unchecked_set_size(this->size() - 1U);
        }

        constexpr void pop_back()
        noexcept(std::is_nothrow_destructible_v<value_type>) {
            if (this->size() > 0) [[likely]] {
                this->unchecked_pop_back();
            }
        }

        template <concepts::compatible_iterator<value_type> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr void unchecked_append_range(Iter first, Sentinel last)
        noexcept(noexcept(storage_.unchecked_append_range(first, last)))
        requires requires () { storage_.unchecked_append_range(first, last); } {
            storage_.unchecked_append_range(first, last);
        }

        template <concepts::compatible_range<value_type> Rng>
        constexpr void unchecked_append_range(Rng&& rng)
        noexcept(noexcept(storage_.unchecked_append_range(rng)))
        requires requires () { storage_.unchecked_append_range(std::forward<Rng>(rng)); } {
            storage_.unchecked_append_range(std::forward<Rng>(rng));
        }

        template <concepts::compatible_iterator<value_type> Iter,
                  concepts::sentinel_or_iter<Iter> Sentinel>
        constexpr void append_range(Iter first, Sentinel last) {
            const std::size_t size { std::distance(first, last) };
            if (size + this->size() > N) [[unlikely]] {
                throw std::bad_alloc{};
            }
            storage_.unchecked_append_range(first, size);
        }

        template <concepts::compatible_range<value_type> Rng>
        constexpr void append_range(Rng&& rng) {
            const std::size_t size {detail::range_size(rng)};
            if (size + this->size() > N) [[unlikely]] {
                throw std::bad_alloc{};
            }
            storage_.unchecked_append_range(std::ranges::begin(rng), size);
        }

        constexpr void resize(size_type new_size, const_reference value)
        requires std::copy_constructible<value_type> {
            if (new_size == this->size()) [[unlikely]] {
                return;
            } if (new_size > N) [[unlikely]] {
                throw std::bad_alloc{};
            } if (new_size > this->size()) {
                storage_.unchecked_append(new_size - this->size(), value);
            } else {  // new_size < this->size()
                if constexpr (!std::is_trivially_destructible_v<value_type>) {
                    std::destroy_n(this->begin() + new_size, this->size() - new_size);
                }
                storage_.unchecked_set_size(new_size);
            }
        }

        constexpr void resize(size_type new_size)
        requires std::default_initializable<value_type> {
            this->resize(new_size, value_type{});
        }

        constexpr void clear() noexcept(noexcept(storage_.clear())) {
            storage_.clear();
        }

        /****************************** UTILITIES ******************************/

        constexpr void swap(inplace_vector& other)
        noexcept(std::is_nothrow_move_constructible_v<inplace_vector>
              && std::is_nothrow_move_assignable_v<inplace_vector>)
        requires std::movable<value_type> {
            auto temp { std::move(other) };
            other = std::move(*this);
            *this = std::move(temp);
        }

        constexpr friend void swap(inplace_vector& lhs, inplace_vector& rhs)
        noexcept(noexcept(lhs.swap(rhs)))
        requires requires() { lhs.swap(rhs); } {
            lhs.swap(rhs);
        }

        constexpr friend bool operator==(const inplace_vector& x, const inplace_vector& y)
        noexcept(concepts::nothrow_equality_comparable<value_type>)
        requires std::equality_comparable<value_type> {
            return x.size() == y.size() && std::ranges::equal(x, y);
        }

        constexpr friend auto operator<=>(const inplace_vector& lhs, const inplace_vector& rhs)
        noexcept((std::three_way_comparable<value_type>
                && concepts::nothrow_three_way_comparable<value_type>)
            ||  (!std::three_way_comparable<value_type>
                && concepts::nothrow_less_comparable<value_type>))
        requires concepts::less_comparable<value_type> {
            if constexpr (std::three_way_comparable<T>) {
                return std::lexicographical_compare_three_way(lhs.begin(), lhs.end(),
                                                              rhs.begin(), rhs.end());
            } else {
                const auto sz = std::min(lhs.size(), rhs.size());
                for (std::size_t i = 0; i < sz; ++i) {
                    if (lhs[i] < rhs[i]) {
                        return std::strong_ordering::less;
                    } if (rhs[i] < lhs[i]) {
                        return std::strong_ordering::greater;
                    }
                }
                return lhs.size() <=> rhs.size();
            }
        }

    }; // class inplace_vector

} // namespace urlicht

#endif //INPLACE_VECTOR_H
