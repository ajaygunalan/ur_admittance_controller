### **Why Monads?**
Functors lift `T → U`. But when your functions return **wrapped values** (e.g., `T → M<U>`), naive lifting nests wrappers: `M<M<U>>`.

**A monad** adds the ability to **flatten** and to compose such functions:

- `join  : M<M<T>> → M<T>`
- `construct : T → M<T>`
- `mbind : (M<T1>, T1 → M<T2>) → M<T2>`  (aka `bind`, `>>=`)

`mbind` ≈ `transform` + `join`.

**Laws (intuition):**
- Left unit: `mbind(construct(a), f) == f(a)`
- Right unit: `mbind(m, construct) == m`
- Associativity: `mbind(mbind(m, f), g) == mbind(m, [&](auto x){ return mbind(f(x), g); })`

---

### **Chaining wrapped computations**
If `user_full_name : string → M<string>` and `to_html : string → M<string>`:

```cpp
M<std::string> current_user_html(M<std::string> login) {
    return login | mbind(user_full_name)
                 | mbind(to_html);
}
```

No nesting, just straight-line composition.