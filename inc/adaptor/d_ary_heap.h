#ifndef D_ARY_HEAP_H
#define D_ARY_HEAP_H
#include <Containers/inplace_vector.h>
#include "base_heap_.h"
#include "compare.h"

namespace urlicht {

    template <
        typename T,
        size_t d = 4,
        concepts::random_access_container Container = std::vector<T>,
        concepts::comparison_functor<T> Compare = compare::less<>>
    requires std::same_as<T, typename Container::value_type> && std::is_object_v<T> && (d >= 2)
    class d_ary_heap final : public detail::base_heap<T, Container, Compare> {

    public:
        using BaseHeap_ = detail::base_heap<T, Container, Compare>;
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

        static consteval size_type arity() noexcept { return d; }

    private:
        // Assumes nothrow move assignable value_type
        constexpr void heapify_up(size_type from)
        noexcept(std::is_nothrow_invocable_v<Compare, value_type, value_type>) {
            value_type val = std::move(this->data_[from]);
            size_type par = (from - 1) / d;
            while(from && this->compare_(this->data_[par], val)) {
                this->data_[from] = std::move(this->data_[par]);
                from = par;
                par = (from - 1) / d;
            }
            this->data_[from] = std::move(val);
        }

        constexpr void heapify_down(size_type idx)
        noexcept(std::is_nothrow_invocable_v<Compare, value_type, value_type>) {
            const size_type n = this->data_.size();
            value_type val = std::move(this->data_[idx]);

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
                if (!this->compare_(val, this->data_[best])) break;
                this->data_[idx] = std::move(this->data_[best]);
                idx = best;
            }
            this->data_[idx] = std::move(val);
        }

        constexpr void build_heap() noexcept(noexcept(this->heapify_down(size_type {}))) {
            const auto m = this->data_.size();
            if (m <= 1) return;
            for(size_type i = (m - 2) / d + 1U; i-- > 0U;) {
                this->heapify_down(i);
            }
        }

    public:
        template <typename VTy>
        requires std::constructible_from<value_type, VTy>
        constexpr d_ary_heap(std::initializer_list<VTy> init)
            : BaseHeap_(init.begin(), init.end()) {
            this->build_heap();
        }

        //Delegate to the constructors of base_heap
        template<typename ...Arg>
        explicit constexpr d_ary_heap(Arg&&... args)
        : BaseHeap_(std::forward<Arg>(args)...) {
            this->build_heap();
        }

        constexpr d_ary_heap(const d_ary_heap&) = default;
        constexpr d_ary_heap(d_ary_heap&&) = default;
        constexpr d_ary_heap& operator=(const d_ary_heap&) = default;
        constexpr d_ary_heap& operator=(d_ary_heap&&) = default;
        constexpr ~d_ary_heap() = default;

        // Modifiers
        template <typename... Args>
        requires std::constructible_from<T, Args&&...>
        constexpr void emplace(Args&& ...args) {
            this->data_.emplace_back(std::forward<Args>(args)...);
            this->heapify_up(this->data_.size() - 1);
        }

        template <typename Input>
        requires std::constructible_from<T, Input&&>
        constexpr void replace(Input&& value) {
            if(this->empty()) {
                throw std::out_of_range("d_ary_heap::replace(): heap is empty.");
            }
            this->data_[0] = std::forward<Input>(value);
            this->heapify_down(0);
        }

        template <typename Input>
        requires std::constructible_from<T, Input&&>
        constexpr void unchecked_replace(Input&& value)
        noexcept(noexcept(this->heapify_down(0))) {
            this->data_[0] = std::forward<Input>(value);
            this->heapify_down(0);
        }

        constexpr void pop() noexcept {
            this->data_[0] = std::move(this->data_.back());
            this->data_.pop_back();
            if(!this->empty()) {
                this->heapify_down(0);
            }
        }

        template <concepts::compatible_iterator<value_type> InputIt,
                  concepts::sentinel_or_iter<InputIt> Sentinel>
        constexpr size_type push_range(InputIt first, Sentinel last) {
            // Basic exception safety - takes in as many elements as possible until it throws
            const auto n = this->size(); // Original size
            const auto m = static_cast<size_type>(std::distance(first, last)); // No. of new elements
            if constexpr (concepts::reservable_container<Container>) {
                this->data_.reserve(n + m);
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
                for (; first != last; ++first) {
                    this->data_.emplace_back(*first);
                }
                this->build_heap();
            } else {
                //otherwise, heapify one-by-one, O(m log(n + m))
                auto start_pos_ = n;
                for (; first != last; ++first, ++start_pos_) {
                    this->data_.emplace_back(*first);
                    this->heapify_up(start_pos_);
                }
            }
            return m;
        }

        template <concepts::compatible_range<value_type> Rng>
        constexpr void push_range(Rng&& r) {
            if constexpr (concepts::rvalue_range<Rng&&>) {
                this->push_range(
                        std::make_move_iterator(std::ranges::begin(r)),
                        std::make_move_iterator(std::ranges::end(r)));
            } else {
                this->push_range(std::ranges::begin(r), std::ranges::end(r));
            }
        }

        constexpr void merge(d_ary_heap&& other) {
            this->push_range(std::make_move_iterator(other.begin()),
                             std::make_move_iterator(other.end()));
            other.clear();
        }

        // Utilities
        constexpr void clear() noexcept(this->data_.clear()) {
            this->data_.clear();
        }

        constexpr void swap(d_ary_heap& other)
        noexcept(std::is_nothrow_swappable_v<Container> &&
                 std::is_nothrow_swappable_v<Compare>)
        requires std::swappable<Compare> {
            using std::swap;
            swap(this->data_, other.data_);
            if constexpr (!std::is_empty_v<value_compare>) {
                swap(this->compare_, other.compare_);
            }
        }

        friend constexpr void swap(d_ary_heap& lhs, d_ary_heap& rhs)
        noexcept(std::is_nothrow_swappable_v<Container> && std::is_nothrow_swappable_v<Compare>)
        requires std::swappable<Compare> {
            lhs.swap(rhs);
        }

    };

    template <typename T, typename Container = std::vector<T>, typename Compare = compare::less<>>
    using binary_heap = d_ary_heap<T, 2, Container, Compare>;

    template <typename T, typename Container = std::vector<T>>
    using min_heap = d_ary_heap<T, 4, Container, compare::greater<>>;

    template <typename T, typename Container = std::vector<T>>
    using max_heap = d_ary_heap<T, 4, Container, compare::less<>>;

    template <typename T, typename Container = std::vector<T>, typename Compare = compare::less<>>
    using ternary_heap = d_ary_heap<T, 3, Container, Compare>;

    template <typename T, std::size_t N, typename Compare = compare::less<>>
    using static_heap = d_ary_heap<T, 4, inplace_vector<T, N>, Compare>;

    // CTAD
    template <typename T>
    d_ary_heap(std::initializer_list<T>) -> d_ary_heap<T>;

    template <concepts::random_access_container Container>
    d_ary_heap(Container) -> d_ary_heap<typename Container::value_type>;

}
#endif //D_ARY_HEAP_H
