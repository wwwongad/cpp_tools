#ifndef TAGGED_PTR_H
#define TAGGED_PTR_H
#include <memory>
#include <type_traits>
#include <cstdint>

namespace urlicht {

    template<
        typename T,
        typename Deleter = std::default_delete<T>,
        bool IS_OWNING = false>
    class tagged_ptr {
    public:
        using pointer = T*;
        using element_type = T;
        using deleter_type = Deleter;

        static_assert(std::is_object_v<T>, "T must be an object type");
        static_assert(requires (deleter_type deleter, pointer ptr) { deleter(ptr); },
                     "Deleter must be a function object for T*");
        static_assert(alignof(element_type) >= 2, "T must be 2 or more bytes aligned");
        static_assert(!(alignof(element_type) & alignof(element_type) - 1), "Alignment must be a power of 2");

    private:
        using tag_size_type = uint8_t;  // size type to hold the number of free bits

        static constexpr tag_size_type NUM_FREE_BITS = std::bit_width(alignof(element_type)) - 1;
        // ptr_ & PTR_MASK == raw pointer
        static constexpr uintptr_t PTR_MASK = ~((static_cast<uintptr_t>(1) << NUM_FREE_BITS) - 1);
        // ptr_ & TAG_MASK == pointer tag
        static constexpr uintptr_t TAG_MASK = ~PTR_MASK;

        using tag_type =
            std::conditional_t<NUM_FREE_BITS <= 8,  std::uint8_t,
                std::conditional_t<NUM_FREE_BITS <= 16, std::uint16_t,
                    std::conditional_t<NUM_FREE_BITS <= 32, std::uint32_t, std::uint64_t>
                >
            >;

        uintptr_t ptr_{};
        [[no_unique_address]] deleter_type deleter_;

    public:
        /************************* CONSTRUCTORS *************************/
        constexpr tagged_ptr() noexcept = default;

        explicit constexpr tagged_ptr(std::nullptr_t)
        noexcept(std::is_nothrow_default_constructible_v<deleter_type>)
        requires std::default_initializable<deleter_type>
        : deleter_{} {   }

        explicit constexpr tagged_ptr(const pointer ptr) noexcept
        requires std::default_initializable<deleter_type>
        : ptr_{reinterpret_cast<uintptr_t>(ptr)} {   }

        // UB if deleter_type is a rvalue reference but Del is a lvalue reference, or vice versa
        template <typename Del>
        requires std::constructible_from<deleter_type, Del&&>
        constexpr tagged_ptr(const pointer ptr, Del&& del)
        noexcept(std::is_nothrow_constructible_v<deleter_type, Del&&>)
        : ptr_{reinterpret_cast<uintptr_t>(ptr)},  deleter_{std::forward<Del>(del)} {   }

        constexpr tagged_ptr(const tagged_ptr& other)
        noexcept(std::is_nothrow_copy_constructible_v<deleter_type>)
        requires std::copy_constructible<deleter_type> = default;

        constexpr tagged_ptr(tagged_ptr&& other)
        noexcept(std::is_nothrow_move_constructible_v<deleter_type>)
        requires std::move_constructible<deleter_type> = default;

        constexpr tagged_ptr& operator=(const tagged_ptr& other)
        noexcept(std::is_nothrow_copy_assignable_v<deleter_type>)
        requires std::is_copy_assignable_v<deleter_type> = default;

        constexpr tagged_ptr& operator=(tagged_ptr&& other)
        noexcept(std::is_nothrow_move_assignable_v<deleter_type>)
        requires std::is_move_assignable_v<deleter_type> = default;

        constexpr ~tagged_ptr()
        noexcept(!IS_OWNING || noexcept(this->get_deleter()(this->get()))) {
            if constexpr (IS_OWNING) {
                if (get()) {
                    get_deleter()(get());
                }
            }
        }

        /************************* MODIFIERS *************************/

        constexpr pointer release() noexcept {
            const pointer tmp = get();
            ptr_ = 0;
            return tmp;
        }

        // For non-owning tagged_ptr, the pointer is returned
        [[nodiscard]] constexpr pointer reset(pointer new_ptr = pointer{}) noexcept
        requires (!IS_OWNING) {
            auto tmp = get();
            ptr_ = reinterpret_cast<uintptr_t>(new_ptr);
            return tmp;
        }

        // For owning tagged_ptr, the pointer is deleted
        constexpr void reset(pointer new_ptr = pointer{})
        noexcept(noexcept(this->get_deleter()(this->get())))
        requires (IS_OWNING) {
            auto tmp = get();
            ptr_ = reinterpret_cast<uintptr_t>(new_ptr);
            if (tmp) {
                get_deleter()(tmp);
            }
        }

        constexpr void set_tag(const tag_type tag) noexcept {
            ptr_ = ptr_ & PTR_MASK | static_cast<uintptr_t>(tag) & TAG_MASK;
        }

        constexpr void clear_tag() noexcept {
            ptr_ = ptr_ & PTR_MASK;
        }

        /************************* OBSERVERS *************************/

        [[nodiscard]] static consteval tag_size_type num_free_bits() noexcept {
            return NUM_FREE_BITS;
        }

        [[nodiscard]] constexpr uintptr_t raw() const noexcept {
            return ptr_;
        }

        [[nodiscard]] constexpr pointer get() const noexcept {
            return reinterpret_cast<pointer>(ptr_ & PTR_MASK);
        }

        [[nodiscard]] constexpr tag_type tag() const noexcept {
            return static_cast<tag_type>(ptr_ & TAG_MASK);
        }

        [[nodiscard]] constexpr Deleter& get_deleter() noexcept {
            return deleter_;
        }

        [[nodiscard]] constexpr const Deleter& get_deleter() const noexcept {
            return deleter_;
        }

        [[nodiscard]] constexpr explicit operator bool() const noexcept {
            return get() != nullptr;
        }

        // For tagged_ptr<T>
        [[nodiscard]] constexpr element_type& operator*() const noexcept {
            return *get();
        }

        [[nodiscard]] constexpr pointer operator->() const noexcept {
            return get();
        }

        /************************* UTILITIES *************************/

        friend constexpr bool operator==(const tagged_ptr& lhs, const tagged_ptr& rhs) noexcept {
            return lhs.get() == rhs.get();
        }

        friend constexpr bool operator==(const tagged_ptr& lhs, pointer rhs) noexcept {
            return lhs.get() == rhs;
        }

        friend constexpr bool operator==(pointer lhs, const tagged_ptr& rhs) noexcept {
            return lhs == rhs.get();
        }

        friend constexpr bool operator==(const tagged_ptr& lhs, std::nullptr_t) noexcept {
            return lhs.get() == nullptr;
        }

        friend constexpr bool operator==(std::nullptr_t, const tagged_ptr& lhs) noexcept {
            return lhs.get() == nullptr;
        }

        friend constexpr void swap(tagged_ptr& lhs, tagged_ptr& rhs)
        noexcept(std::is_nothrow_swappable_v<deleter_type>) {
            using std::swap;
            swap(lhs.ptr_, rhs.ptr_);
            if constexpr(!std::is_empty_v<deleter_type>) {
                swap(lhs.deleter_, rhs.deleter_);
            }
        }

        friend constexpr std::ostream operator<<(std::ostream& os, const tagged_ptr& ptr) noexcept {
            return os << ptr.get();
        }

    };
}

#endif //TAGGED_PTR_H
