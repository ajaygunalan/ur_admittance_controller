### **`void_t` and SFINAE**
`void_t<...>` is always `void` **if** its arguments are well-formed; otherwise that overload is SFINAE'd away.

**Has `value_type`?**
```cpp
template <class, class = void_t<>>
struct has_value_type : std::false_type {};

template <class C>
struct has_value_type<C, void_t<typename C::value_type>> : std::true_type {};
```

**Is iterable?**
```cpp
template <class, class = void_t<>>
struct is_iterable : std::false_type {};

template <class C>
struct is_iterable<C,
  void_t<decltype(*begin(std::declval<C>())), decltype(end(std::declval<C>()))>>
: std::true_type {};
```

---

### **Dispatch with `constexpr if`**
```cpp
template <class C>
auto sum(const C& c) {
    if constexpr (has_value_type<C>())       return sum_collection(c);
    else if constexpr (is_iterable<C>())     return sum_iterable(c);
    else                                      static_assert(sizeof(C)==0, "Not a collection");
}
```
