namespace urlicht {

    template <
        typename T,
        size_t d = 4,
        concepts::random_access_container Container = std::vector<T>,
        concepts::compatible_functor<T> Compare = compare::less<>>
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

        static consteval size_t arity() noexcept { return d; }

    private:
        constexpr void heapify_up(size_t from) noexcept {
            T val = std::move(this->data_[from]);
            size_t par = (from - 1) / d;
            while(from && this->compare_(this->data_[par], val)) {
                this->data_[from] = std::move(this->data_[par]);
                from = par;
                par = (from - 1) / d;
            }
            this->data_[from] = std::move(val);
        }

        constexpr void heapify_down(size_t idx) noexcept {
            const size_t n = this->data_.size();
            T val = std::move(this->data_[idx]);

            while (true) {
                size_t ch = d * idx + 1;
                if (ch >= n) break;

                size_t best = ch;
                if constexpr (d == 2) {
                    best += static_cast<size_t>(ch + 1 < n && this->compare_(this->data_[ch], this->data_[ch + 1]));
                }
                else {
                    const size_t last = std::min(d * idx + d, n - 1);
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

        constexpr void build_heap() noexcept {
            const auto m = this->data_.size();
            if (m <= 1) return;
            for(size_t i = (m - 2) / d + 1U; i-- > 0U;) {
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
        template <typename Input>
        requires std::constructible_from<T, Input&&>
        constexpr void push(Input&& value) {
            this->emplace(std::forward<Input>(value));
        }

        template <typename... Args>
        requires std::constructible_from<T, Args&&...>
        constexpr void emplace(Args&& ...args) {
            this->data_.emplace_back(std::forward<Args>(args)...);
            heapify_up(this->data_.size() - 1);
        }

        template <typename Input>
        requires std::constructible_from<T, Input&&>
        constexpr void replace(Input&& value) {
            if(this->empty()) {
                throw std::out_of_range("d_ary_heap::replace(): heap is empty.");
            }
            this->data_[0] = std::forward<Input>(value);
            heapify_down(0);
        }

        template <typename Input>
        requires std::constructible_from<T, Input&&>
        constexpr void unchecked_replace(Input&& value) noexcept {
            this->data_[0] = std::forward<Input>(value);
            heapify_down(0);
        }

        constexpr void pop() noexcept {
            this->data_[0] = std::move(this->data_.back());
            this->data_.pop_back();
            if(!this->empty()) {
                heapify_down(0);
            }
        }

        template <std::input_iterator InputIt>
        requires std::constructible_from<value_type, std::iter_reference_t<InputIt>>
        constexpr void push_range(InputIt first, InputIt last) {
            // Basic exception safety - takes in as many elements as possible until it throws
            if constexpr (std::random_access_iterator<InputIt>) {
                const size_t n = this->size(), m = static_cast<size_t>(last - first);
                if constexpr (concepts::reservable_container<Container>) {
                    this->data_.reserve(n + m);
                }

                auto logdn = [](const size_t x) -> size_t { // an approximate value of logd(n)
                    if (x < d) return 1;
                    if constexpr (d == 2) {
                        return static_cast<size_type>(std::bit_width(x) - 1);
                    }
                    return static_cast<size_type>(std::bit_width(x) / std::bit_width(d));
                };

                //rebuild if m >= (n / max(1, log2(n))), O(n + m)
                if (!n || m >= n / logdn(n)) {
                    for (; first != last; ++first) {
                        this->data_.emplace_back(std::forward<decltype(*first)>(*first));
                    }
                    this->data_.
                    build_heap();
                    return;
                }
            }
            //otherwise, or for input iterator, heapify one-by-one, O(m log(n + m))
            size_type start_pos_ = this->size();
            for (; first != last; ++first, ++start_pos_) {
                this->data_.emplace_back(std::forward<decltype(*first)>(*first));
                heapify_up(start_pos_);
            }
        }

        template <std::ranges::input_range Rng>
        requires std::constructible_from<value_type, std::ranges::range_reference_t<Rng>>
        constexpr void push_range(Rng&& r) {
            if constexpr (std::is_rvalue_reference_v<decltype(r)>) {
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
        requires std::swappable<Container> && std::swappable<Compare> {
            using std::swap;
            swap(this->data_, other.data_);
            if constexpr (!std::is_empty_v<value_compare>) {
                swap(this->compare_, other.compare_);
            }
        }

        friend constexpr void swap(d_ary_heap& lhs, d_ary_heap& rhs)
        noexcept(std::is_nothrow_swappable_v<Container> && std::is_nothrow_swappable_v<Compare>)
        requires std::swappable<Container> && std::swappable<Compare> {
            lhs.swap(rhs);
        }

        friend constexpr bool operator== (const d_ary_heap& lhs, const d_ary_heap& rhs)
        noexcept(concepts::nothrow_equality_comparable<value_type>)
        requires concepts::equality_comparable<value_type> {
            return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
        }

        constexpr friend auto operator<=>(const d_ary_heap& lhs, const d_ary_heap& rhs)
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
    };

    template <typename T, typename Container = std::vector<T>, typename Compare = compare::less<>>
    using binary_heap = d_ary_heap<T, 2, Container, Compare>;

    template <typename T, typename Container = std::vector<T>>
    using min_heap = d_ary_heap<T, 4, Container, compare::greater<>>;

    template <typename T, typename Container = std::vector<T>>
    using max_heap = d_ary_heap<T, 4, Container, compare::less<>>;

    template <typename T, typename Container = std::vector<T>, typename Compare = compare::less<>>
    using ternary_heap = d_ary_heap<T, 3, Container, Compare>;

}
#endif //D_ARY_HEAP_H
