### **`std::visit` with an overloaded callable**
Helper:
```cpp
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
```

Use:
```cpp
void point_for(player who){
    std::visit(overloaded{
        [&](const normal_scoring& s){ /* bump score or switch to forty */ },
        [&](const forty_scoring&  s){ /* win / to deuce / advance other */ },
        [&](const deuce&){            /* to advantage(who) */ },
        [&](const advantage& s){      /* win or back to deuce */ }
    }, state);
}
```

- Pattern coverage is **checked** by the compiler (types must be handled).
- Prefer generic lambdas where args vary.

