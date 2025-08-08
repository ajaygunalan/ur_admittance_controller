### **See what the compiler deduced**
```cpp
template <typename T> class error;         // no definition
error< contained_type_t<std::vector<std::string>> >(); // forces type in error
```
Or assert it:
```cpp
static_assert(std::is_same_v<int, contained_type_t<std::vector<int>>>);
```

---

### **Recreate `is_same` (pattern matching via specialization)**
```cpp
template <class, class> struct is_same : std::false_type {};
template <class T>      struct is_same<T, T> : std::true_type {};
```

### **Recreate `remove_reference_t`**
```cpp
template <class T>      struct remove_reference      { using type = T; };
template <class T>      struct remove_reference<T&>  { using type = T; };
template <class T>      struct remove_reference<T&&> { using type = T; };
template <class T> using remove_reference_t = typename remove_reference<T>::type;
```

> Template **partial specialization** is compile-time pattern matching.
