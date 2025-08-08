### **Attach reply channel to each message**
Carry the client socket along the pipeline and keep transformations pure.

```cpp
template <typename T>
struct with_client {
    T value;
    tcp::socket* socket;
    void reply(const std::string& msg) const {
        auto sp = std::make_shared<std::string>(msg);
        boost::asio::async_write(*socket, boost::asio::buffer(*sp), [sp](auto,auto){});
    }
};
```

- Make `service` emit `with_client<std::string>`.
- Lift transforms/filters to work **inside** `with_client`.
```cpp
auto transform = [](auto f){
  return reactive::operators::transform(lift_with_client(f));
};
auto filter = [](auto p){
  return reactive::operators::filter(apply_with_client(p));
};
```

**Final server sketch**
```cpp
auto pipeline =
  service(io)
  | transform(trim)
  | filter([](const std::string& s){ return !s.empty() && s[0] != '#'; })
  | transform([](const std::string& s){ return mtry([&]{ return json::parse(s); }); })
  | transform([](const auto& exp){ return mbind(exp, bookmark_from_json); })
  | sink([](const auto& msg){
        const auto exp = msg.value;
        if (!exp) { msg.reply("ERROR: Request not understood\n"); return; }
        if (exp->text.find("C++") != std::string::npos)
            msg.reply("OK: " + to_string(exp.get()) + "\n");
        else
            msg.reply("ERROR: Not a C++-related link\n");
    });
```

**Nesting intuition**: stream (monad) of `with_client` (monad) of `expected<T,E>` (monad). Lift/bind at the right layer; keep the rest unchanged.
