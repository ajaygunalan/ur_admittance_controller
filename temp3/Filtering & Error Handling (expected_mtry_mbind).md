### **Filter**
Pass through only messages that satisfy a predicate.
```cpp
namespace detail {
  template <typename Sender, typename Pred, typename Msg = typename Sender::value_type>
  class filter_impl {
  public:
    using value_type = Msg;
    void process_message(Msg&& m) const {
      if (std::invoke(m_pred, m)) m_emit(std::move(m));
    }
  private:
    Sender m_sender; Pred m_pred;
    std::function<void(Msg&&)> m_emit;
  };
}
```

---

### **Parse JSON and validate using `expected`**
- Use `mtry` to adapt exception-throwing code to `expected<T, std::exception_ptr>`.
- Use `mbind` to chain fallible steps without `if` ladders.

```cpp
auto pipeline =
  service(io)
  | transform(trim)
  | filter([](const std::string& s){ return !s.empty() && s[0] != '#'; })

  | transform([](const std::string& s){         // expected<json, exception_ptr>
        return mtry([&]{ return json::parse(s); });
    })

  | transform([](const auto& exp_json){         // mbind to convert json -> bookmark
        return mbind(exp_json, bookmark_from_json);
    })

  | sink_to_cerr;
```
`bookmark_from_json(const json&) -> expected<bookmark_t, std::exception_ptr>`
