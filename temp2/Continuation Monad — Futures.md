### **Futures as Functor/Monad**
A `future<T>` is a **handle** to a value available later.

**Functor:** transform the eventual value when it arrives.
```cpp
get_page(url) | transform(extract_title); // future<string>
```

**Monad:** chain async steps without nesting `future<future<T>>`.

```cpp
template <typename T, typename F>
auto mbind(const future<T>& fut, F f) {
    return fut.then([=](future<T> ready){
        return f(ready.get()); // returns future<U>
    });
}
```

**Result:**
```cpp
return current_user() | mbind(user_full_name)
                      | mbind(to_html); // future<string>
```

> `std::future` blocks on `.get()`. Use continuations (`.then`) from Concurrency TS / `boost::future` / Folly to keep it non-blocking.