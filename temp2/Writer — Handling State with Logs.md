### **Pure State via a Log (Writer-like)**
Accumulate **context** (a log) alongside values without side effects.

```cpp
template <typename T>
class with_log {
public:
    with_log(T v, std::string log = {}) : v_(std::move(v)), log_(std::move(log)) {}
    const T& value() const { return v_; }
    const std::string& log() const { return log_; }
private:
    T v_;
    std::string log_;
};
```

`mbind`: append logs as you compose.

```cpp
template <typename T, typename F,
          typename Ret = typename std::result_of<F(T)>::type>
Ret mbind(const with_log<T>& w, F f) {
    auto r = f(w.value());
    return Ret(r.value(), w.log() + r.log());
}
```

- Each chain has its **own log** (no interleaving).
- Works naturally with async chains, too.