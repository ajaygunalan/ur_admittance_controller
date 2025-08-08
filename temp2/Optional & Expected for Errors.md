### **`std::optional<T>`**
- Sum of **T or nothing** → explicit "maybe" results.
- Prefer over raw pointers for "missing value" semantics.

```cpp
template <class T, class V>
std::optional<T> get_if_opt(const V& v) {
    if (auto* p = std::get_if<T>(&v)) return *p; else return std::nullopt;
}
```

---

### **`expected<T,E>` (value **or** error)**
- When failure should carry **info** (message/code/exception).

Sketch:
```cpp
template<class T, class E>
class expected {
    union { T val; E err; };
    bool ok;
public:
    static expected success(T v){ expected r; r.ok=true;  new(&r.val) T(std::move(v)); return r; }
    static expected error  (E e){ expected r; r.ok=false; new(&r.err) E(std::move(e)); return r; }

    ~expected(){ ok ? val.~T() : err.~E(); }

    T&       get(){ if(!ok) throw std::logic_error("no value"); return val; }
    const E& error() const { if(ok)  throw std::logic_error("no error"); return err; }

    explicit operator bool() const { return ok; }
};
```

**Use:** return `expected<T,E>` from functions that can fail. Easier to **pass across threads** than exceptions; more informative than `optional`.

