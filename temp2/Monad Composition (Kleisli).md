### **Compose monadic functions directly**
Given `f : A → M<B>` and `g : B → M<C>`, make `A → M<C>`:

```cpp
template <typename F, typename G>
auto mcompose(F f, G g) {
    return [=](auto a) {
        return mbind(f(a), g);
    };
}
```

Examples:
```cpp
auto user_html = mcompose(user_full_name, to_html);
auto grandchildren = mcompose(children, children); // ranges/vectors
```

**Laws (nice form):**
- `mcompose(f, construct) == f`
- `mcompose(construct, f) == f`
- Associativity: `mcompose(f, mcompose(g, h)) == mcompose(mcompose(f, g), h)`