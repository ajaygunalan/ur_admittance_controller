### **`std::optional<T>` as a Monad**
Chain computations that can fail without manual `if` ladders:

```cpp
template <typename T, typename F>
auto mbind(const std::optional<T>& opt, F f)
    -> decltype(f(*opt))
{
    if (opt) return f(*opt);
    return {};
}
```

Usage:
```cpp
return current_login | mbind(user_full_name)
                     | mbind(to_html);
```

---

### **`expected<T,E>`: return value **or** error**
Propagate the **first error**; otherwise pass the value forward.

```cpp
template <typename T, typename E, typename F,
          typename Ret = typename std::result_of<F(T)>::type>
Ret mbind(const expected<T,E>& exp, F f) {
    if (!exp) return Ret::error(exp.error());
    return f(exp.value());
}
```

---

### **Try monad (wrap exceptions in `expected`)**
Bridge exception-throwing code to monadic style:

```cpp
template <typename F,
          typename R = typename std::result_of<F()>::type>
expected<R, std::exception_ptr> mtry(F f) {
    try { return expected<R,std::exception_ptr>::success(f()); }
    catch (...) { return expected<R,std::exception_ptr>::error(std::current_exception()); }
}
```

And the other way around:
```cpp
template <typename T>
T get_or_throw(const expected<T, std::exception_ptr>& e) {
    if (e) return e.value();
    std::rethrow_exception(e.error());
}
```