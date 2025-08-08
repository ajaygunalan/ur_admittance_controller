### **Streams behave like monads**
Emits **0..N** values over time. We’ll provide **functor** (`transform`) and **monad** (`construct`, `join`) ops.

---

### **Sink (consumer)**
Call a function for each incoming message.
```cpp
namespace detail {
  template <typename Sender, typename Fn, typename Msg = typename Sender::value_type>
  class sink_impl {
  public:
    sink_impl(Sender&& s, Fn f) : m_sender(std::move(s)), m_fn(f) {
      m_sender.set_message_handler([this](Msg&& m){ process_message(std::move(m)); });
    }
    void process_message(Msg&& m) const { std::invoke(m_fn, std::move(m)); }
  private:
    Sender m_sender; Fn m_fn;
  };
}
template <typename Sender, typename Fn>
auto sink(Sender&& s, Fn&& f) {
  return detail::sink_impl<Sender, Fn>(std::forward<Sender>(s), std::forward<Fn>(f));
}
```

Pipe helper:
```cpp
namespace detail { template <typename Fn> struct sink_helper { Fn f; }; }
template <typename Sender, typename Fn>
auto operator|(Sender&& s, detail::sink_helper<Fn> h) {
  return detail::sink_impl<Sender, Fn>(std::forward<Sender>(s), h.f);
}
// usage
auto sink_to_cerr = sink([](const auto& msg){ std::cerr << msg << "\n"; });
service(io) | sink_to_cerr;
```

---

### **Transform (functor `map`)**
```cpp
namespace detail {
  template <typename Sender, typename F,
            typename In = typename Sender::value_type,
            typename Out = decltype(std::declval<F>()(std::declval<In>()))>
  class transform_impl {
  public:
    using value_type = Out;
    transform_impl(Sender&& s, F f) : m_sender(std::move(s)), m_f(f) {}
    template <typename EmitFn>
    void set_message_handler(EmitFn emit){
      m_emit = emit;
      m_sender.set_message_handler([this](In&& m){ process_message(std::move(m)); });
    }
    void process_message(In&& m) const { m_emit(std::invoke(m_f, std::move(m))); }
  private: Sender m_sender; F m_f; std::function<void(Out&&)> m_emit;
  };
}
```

---

### **values (monadic constructor)**
```cpp
template <typename T>
class values {
public:
  using value_type = T;
  explicit values(std::initializer_list<T> vs) : m_values(vs) {}
  template <typename EmitFn>
  void set_message_handler(EmitFn emit){
    for (auto v : m_values) emit(std::move(v));
  }
private:
  std::vector<T> m_values;
};
```

---

### **join (flatten a stream-of-streams)**
```cpp
namespace detail {
  template <typename Sender,
            typename Src = typename Sender::value_type,
            typename Out = typename Src::value_type>
  class join_impl {
  public:
    using value_type = Out;
    template <typename EmitFn>
    void set_message_handler(EmitFn emit){ m_emit = emit;
      m_sender.set_message_handler([this](Src&& s){ process_message(std::move(s)); });
    }
    void process_message(Src&& s){
      m_sources.emplace_back(std::move(s));
      m_sources.back().set_message_handler(m_emit);
    }
  private:
    Sender m_sender;
    std::function<void(Out&&)> m_emit;
    std::list<Src> m_sources; // keep children alive
  };
}
```

**Example: multiple ports → one stream**
```cpp
auto pipeline =
  values{42042, 42043, 42044}
  | transform([&](int port){ return service(io, port); })
  | join()
  | sink_to_cerr;
```
