### **Goal**
Ergonomic, all-or-nothing record updates:
```cpp
with(martha)(
    name = "Martha",
    surname = "Jones",
    age = 42
);
```

### **AST Nodes**
- `field<Member>` — wraps a member pointer or setter pointer
- `update<Member,Value>` — result of `field::operator=(Value)`
- `transaction<Record>` — functor returned by `with(record)`

**`update::operator()` — 3 cases via `constexpr if`:**
```cpp
template <class Record>
bool operator()(Record& r){
    if constexpr (std::is_invocable_r_v<bool, Member, Record, Value>) {
        return std::invoke(member, r, value);             // setter that may fail
    } else if constexpr (std::is_invocable_v<Member, Record, Value>) {
        std::invoke(member, r, value); return true;       // void setter
    } else {
        std::invoke(member, r) = value; return true;      // data member
    }
}
```

**`transaction::operator()` — copy-and-swap**
```cpp
template <class... Updates>
bool operator()(Updates... us){
    auto tmp = m_record;
    if (all(us(tmp)...)) { std::swap(m_record, tmp); return true; }
    return false;
}

template <class... Bools>
bool all(Bools... bs) const { return (... && bs); } // fold-expression
```
