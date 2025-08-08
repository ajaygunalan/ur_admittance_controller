### **Metaprogramming goal**
Work with **types** at compile time: select implementations, compute return types, etc.

---

### **Get the contained element type**
```cpp
// naive: requires default-constructible T and yields refs/const
template <typename T>
using contained_type_bad = decltype(*begin(T()));

// robust: works for non-default-constructibles and strips cv/ref
template <typename T>
using contained_type_t =
    std::remove_cv_t<
        std::remove_reference_t<
            decltype(*begin(std::declval<T>()))
        >
    >;
```

**Use case — generic sum over an iterable:**
```cpp
template <typename C,
          typename R = contained_type_t<C>>
R sum_iterable(const C& c) {
    return std::accumulate(begin(c), end(c), R{});
}
```

---

### **Prefer `value_type` when a container provides it**
```cpp
template <typename C,
          typename R = typename C::value_type>
R sum_collection(const C& c) {
    return std::accumulate(begin(c), end(c), R{});
}
```

> Containers should expose **`value_type`**; it’s clearer and handles proxy iterators.
