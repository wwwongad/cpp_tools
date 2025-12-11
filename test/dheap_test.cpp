#include <cassert>
#include <iostream>
#include <format>
#include <deque>
#include <forward_list>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <ranges>
#include "Adapters/d_ary_heap.h"

/*
// define for compile-time evaluated containers such as inplace_vector
#ifndef TEST_TARGET_IS_CONSTEXPR
#define TEST_TARGET_IS_CONSTEXPR
#endif
*/

#include <bits/ranges_algo.h>

#include "macro_utils.h"

using namespace urlicht;

void test_arity() {
    static_assert(d_ary_heap<int>::arity() == 4);  // arity defaults to 4 for max performance
    static_assert(d_ary_heap<int, 2>::arity() == 2);
    static_assert(d_ary_heap<int, 3>::arity() == 3);
    static_assert(d_ary_heap<int, 4>::arity() == 4);
    static_assert(d_ary_heap<int, 5>::arity() == 5);
    static_assert(binary_heap<int>::arity() == 2);
    static_assert(ternary_heap<int>::arity() == 3);
    std::cout << "test_arity() passed.\n";
}

/*********************** TESTS FOR CONSTRUCTORS ***********************/

void test_default_init() {
    UL_TEST_CONSTEXPR d_ary_heap<int> heap;
    UL_ASSERT(heap.empty());
    UL_ASSERT(heap.size() == 0);
    std::cout << "test_default_init() passed.\n";
}

void test_il_init() { // Initializes by std::initializer_list
    auto il = {1, 2, 3, 4, 5, 4, 3, 2, 1, -1};
    // default d_ary_heap
    UL_TEST_CONSTEXPR d_ary_heap<int> dheap{il};
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == il.size());
    UL_ASSERT(dheap.top() == 5);    // max_heap by default

    std::cout << "test_il_init() passed.\n";
}

struct NDCmp { // stands for non-default initializable value_compare
    int key;
    NDCmp() = delete;
    explicit NDCmp(const int key) : key(key) {}
    bool operator()(const int a, const int b) const noexcept {
        return a % key > b % key;
    }
};

// Initializes only the non-default-initializable comparator (typename d_ary_heap::value_compare)
void test_ndefault_cmp_init() {
    using cmp_heap = d_ary_heap<int, 4, std::vector<int>, NDCmp>;
    // default constructor requires default initializable value_compare
    static_assert(!std::default_initializable<cmp_heap::BaseHeap_>);

    UL_TEST_CONSTEXPR cmp_heap heap{ NDCmp(100) };
    UL_ASSERT(heap.empty());    // should still be empty
    UL_ASSERT(heap.size() == 0);
    UL_ASSERT(heap.get_value_compare().key == 100);
    std::cout << "test_comp_init() passed.\n";
}

// Iterator Constructors

// Iterators as direct parameters of the container's constructor
void test_iter_init_empty() {
    std::vector<int> v;

    UL_TEST_CONSTEXPR d_ary_heap<int> heap{v.begin(), v.end()};
    UL_ASSERT(heap.empty());
    UL_ASSERT(heap.size() == 0);

    std::cout << "test_iter_init_empty() passed.\n";
}
void test_iter_init_compatible() {
    std::vector<std::string> values{"aaa", "bbb", "ccc", "bbb", "aaa"};

    using heap_t = d_ary_heap<std::string>;
    UL_TEST_CONSTEXPR heap_t dheap(values.begin(), values.end());

    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == values.size());
    UL_ASSERT(dheap.top() == "ccc");

    std::cout << "test_iter_init_compatible() passed.\n";
}

// Same as the above, except that value_compare is not default initializable
void test_iter_init_compatible_ndefault_cmp() {
    std::vector<int> values{1, 2, 3, 4, 5, 4, 3, 2, 1, -1};

    using heap_t = d_ary_heap<int, 4, std::vector<int>, NDCmp>;
    static_assert(!std::constructible_from<heap_t::BaseHeap_,
            std::vector<int>::iterator, std::vector<int>::iterator>);

    UL_TEST_CONSTEXPR heap_t dheap(values.begin(), values.end(), NDCmp(100));
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == values.size());
    UL_ASSERT(dheap.top() == -1);
    UL_ASSERT(dheap.get_value_compare().key == 100);

    std::cout << "test_iter_init_compatible_ndefault_cmp() passed.\n";
}

// Minimum container satisfying the heapable_container concept constraint.
// This does not have an iterator constructor and range constructor
template <typename T>
class simple_vec {
    T data_[100]{};
    std::size_t size_{}; // use size_t to match size_type
public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = T*;
    using const_iterator = const T*;

    // Element access
    reference operator[](size_type index) noexcept { return data_[index]; }
    const_reference operator[](size_type index) const noexcept { return data_[index]; }
    reference front() noexcept { return data_[0]; }
    const_reference front() const noexcept { return data_[0]; }
    reference back() noexcept { return data_[size_ - 1]; }
    const_reference back() const noexcept { return data_[size_ - 1]; }

    // Iterators
    iterator begin() noexcept { return data_; }
    const_iterator begin() const noexcept { return data_; }
    const_iterator cbegin() const noexcept { return data_; }
    iterator end() noexcept { return data_ + size_; }
    const_iterator end() const noexcept { return data_ + size_; }
    const_iterator cend() const noexcept { return data_ + size_; }

    // Capacity
    static constexpr size_type max_size() noexcept { return 100; }
    size_type size() const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }

    // Modifiers
    void push_back(const T& value) noexcept { data_[size_++] = value; }
    void push_back(T&& value) noexcept { data_[size_++] = std::move(value); }
    template <class... Args>
    reference emplace_back(Args&&... args) noexcept {
        new (data_ + size_) T(std::forward<Args>(args)...);
        return data_[size_++];
    }
    void pop_back() noexcept { --size_; }
    void clear() noexcept { size_ = 0; }
};


// For iterators unable to be directly passed as the parameters of the container's constructor
// If d_ary_heap::value_type can be constructed from iterator::value_type,
// then d_ary_heap can be constructed from the iterators.
void test_iter_init_incompatible() {
    std::vector<std::string_view> values{"aaa", "bbb", "ccc", "bbb", "aaa"};

    using heap_t = d_ary_heap<std::string, 4, simple_vec<std::string>>;
    UL_TEST_CONSTEXPR heap_t dheap(values.begin(), values.end());
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == values.size());
    UL_ASSERT(dheap.top() == "ccc");

    std::cout << "test_iter_init_incompatible() passed.\n";
}

// d_ary_heap is constructible with sentinel end iterators, which are of a different type as the beginning
void test_iter_init_incompatible_sentinel_end() {
    auto rng =
              std::views::iota(0, 1000)
            | std::views::filter([](int x){ return x % 3 == 0; })
            | std::views::take(100);


    EXPECT_VALUES_DIFF_TYPE(std::ranges::begin(rng), std::ranges::end(rng));
    static_assert(std::sentinel_for<decltype(std::ranges::end(rng)), decltype(std::ranges::begin(rng))>);

    // Currently, std::vector cannot be constructed with sentinel iterators
    UL_TEST_CONSTEXPR d_ary_heap<int> dheap(std::ranges::begin(rng), std::ranges::end(rng));
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == std::ranges::distance(rng));
    UL_ASSERT(dheap.top() == *std::ranges::max_element(std::ranges::begin(rng),
                                                        std::ranges::end(rng)));

    std::cout << "test_iter_init_incompatible_sentinel_end() passed.\n";
}

void test_iter_init_incompatible_move() {
    std::vector<std::string> values{"aaa", "bbb", "ccc", "bbb", "aaa"};

    // Not directly constructible
    static_assert(!std::constructible_from<simple_vec<std::string>,
        std::vector<std::string>::iterator, std::vector<std::string>::iterator>);

    using heap_t = d_ary_heap<std::string, 4, simple_vec<std::string>>;
    UL_TEST_CONSTEXPR heap_t dheap(std::make_move_iterator(values.begin()),
                                   std::make_move_iterator(values.end()));
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == values.size());
    UL_ASSERT(dheap.top() == "ccc");

    for (const auto & value : values) {
        assert(value.empty()); // Data are successfully moved from
    }

    std::cout << "test_iter_init_incompatible_move() passed.\n";
}

void test_iter_init_incompatible_ndefault_cmp() {
    std::vector values{1, 2, 3, 4, 5, 4, 3, 2, 1, -1};

    using heap_t = d_ary_heap<int, 4, simple_vec<int>, NDCmp>;

    // Not directly constructible
    static_assert(!std::constructible_from<simple_vec<int>,
            std::vector<int>::iterator, std::vector<int>::iterator>);

    // Cannot be constructed without the comparator provided
    static_assert(!std::constructible_from<heap_t::BaseHeap_,
            std::vector<int>::iterator, std::vector<int>::iterator>);

    UL_TEST_CONSTEXPR heap_t dheap(values.begin(), values.end(), NDCmp(100));
    UL_ASSERT(!dheap.empty());
    UL_ASSERT(dheap.size() == values.size());
    UL_ASSERT(dheap.top() == -1);
    UL_ASSERT(dheap.get_value_compare().key == 100);

    std::cout << "test_iter_init_incompatible_ndefault_cmp() passed.\n";
}

// Range Constructors

void test_rng_init_empty() {
    std::vector<int> empty;

    UL_TEST_CONSTEXPR d_ary_heap<int> dheap{empty};
    UL_ASSERT(dheap.empty());
    UL_ASSERT(dheap.size() == 0);

    std::cout << "test_rng_init_empty() passed.\n";
}

void test_rng_init_contiguous() {
    // vector
    std::vector<int> v{3, 1, 4, 1, 5, 9};
    UL_TEST_CONSTEXPR d_ary_heap<int> h1{v};
    UL_ASSERT(!h1.empty());
    UL_ASSERT(h1.size() == v.size());
    UL_ASSERT(h1.top() == 9);

    // string_view
    std::string_view sv = "helloworld";
    UL_TEST_CONSTEXPR d_ary_heap<char> h2{sv};
    UL_ASSERT(!h2.empty());
    UL_ASSERT(h2.size() == sv.size());
    UL_ASSERT(h2.top() == 'w');

    // span
    std::span sp(v.data(), v.size());
    UL_TEST_CONSTEXPR d_ary_heap<int> h3{sp};
    UL_ASSERT(!h3.empty());
    UL_ASSERT(h3.size() == sp.size());
    UL_ASSERT(h3.top() == 9);

    std::cout << "test_rng_init_contiguous() passed.\n";
}

void test_rng_init_noncontiguous() {
    std::list lst{7, 1, 9, 9, 3, 2, 8};
    UL_TEST_CONSTEXPR d_ary_heap<int> h1{lst};
    UL_ASSERT(!h1.empty());
    UL_ASSERT(h1.size() == lst.size());
    UL_ASSERT(h1.top() == 9);

    std::forward_list flst{5, 10, -3, 10, 0};
    UL_TEST_CONSTEXPR d_ary_heap<int> h2{flst};
    UL_ASSERT(!h2.empty());
    UL_ASSERT(h2.size() == 5);
    UL_ASSERT(h2.top() == 10);

    std::set s{1, 2, 3, 4, 5, 6};
    UL_TEST_CONSTEXPR d_ary_heap<int> h3{s};
    UL_ASSERT(!h3.empty());
    UL_ASSERT(h3.size() == s.size());
    UL_ASSERT(h3.top() == 6);

    std::cout << "test_rng_init_noncontiguous() passed.\n";
}

void test_rng_init_pairs() {
    std::map<int, int> mp{{2, 20}, {5, -50}, {-10, 100}, {1, 10}};
    using P = std::pair<int, int>;

    UL_TEST_CONSTEXPR d_ary_heap<P> h{mp};
    UL_ASSERT(!h.empty());
    UL_ASSERT(h.size() == mp.size());
    // Compares the first element of the pair by default
    UL_ASSERT(h.top().first == 5);
    UL_ASSERT(h.top().second == -50);

    std::cout << "test_rng_init_pairs() passed.\n";
}

void test_rng_init_take_views() {
    auto rng = std::views::iota(0, 500)
             | std::views::transform([](int x){ return x * x - 10; })
             | std::views::filter([](int x){ return x % 3 == 0; })
             | std::views::take(100);

    // take_view uses a sentinel for end iterator
    EXPECT_VALUES_DIFF_TYPE(std::ranges::begin(rng), std::ranges::end(rng));
    static_assert(std::sentinel_for<decltype(std::ranges::end(rng)), decltype(std::ranges::begin(rng))>);

    UL_TEST_CONSTEXPR d_ary_heap<int> h{rng};
    UL_ASSERT(!h.empty());
    UL_ASSERT(h.size() == std::ranges::distance(rng.begin(), rng.end()));
    UL_ASSERT(h.top() == *std::ranges::max_element(rng.begin(), rng.end()));

    std::cout << "test_rng_init_take_views() passed.\n";
}

void test_all() {
    /*  Static Methods  */
    test_arity();
    /*  Constructors  */
    test_default_init();
    test_il_init();
    test_ndefault_cmp_init();
    // Iter constructors
    test_iter_init_empty();
    test_iter_init_compatible();
    test_iter_init_compatible_ndefault_cmp();
    test_iter_init_incompatible();
    test_iter_init_incompatible_move();
    test_iter_init_incompatible_ndefault_cmp();
    // Range constructors
    test_rng_init_empty();
    test_rng_init_contiguous();
    test_rng_init_noncontiguous();
    test_rng_init_pairs();
    test_rng_init_take_views();
}



int main() {
    test_all();
}