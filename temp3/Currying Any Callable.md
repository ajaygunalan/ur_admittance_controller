### **Goal**
Turn `f(args...)` into a function object that can be called with **some** args now and the rest later.

---

### **Curried function object**
```cpp
template <class F, class... Captured>
class curried {
    F f_;
    std::tuple<Captured...> cap_;
    template <class... A>
    static auto pack(A&&... a) {
        return std::tuple<std::decay_t<A>...>(std::forward<A>(a)...);
    }
public:
    curried(F f, Captured... cap)
        : f_(std::move(f)), cap_(pack(std::move(cap)...)) {}

    template <class... New>
    auto operator()(New&&... a) const {
        auto new_cap = pack(std::forward<New>(a)...);
        auto all     = std::tuple_cat(cap_, std::move(new_cap));

        if constexpr (std::is_invocable_v<F, Captured..., New...>) {
            return std::apply(f_, all);               // call now
        } else {
            return curried<F, Captured..., New...>(f_, all); // need more args
        }
    }
};
```
Use:
```cpp
auto print_person_cd = curried{print_person};
print_person_cd(martha)(std::cout)(person_t::full_name);
```

> Store by **value**; pass refs via `std::ref`/`std::cref` when needed.
