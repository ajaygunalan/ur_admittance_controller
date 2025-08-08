### **What’s a Functor (FP meaning)**
A class template `F<T>` is a **functor** if it supports a `transform` (aka `map`) that lifts a function `T1 → T2` to operate on `F<T1>` and produce `F<T2>`.

**Laws (intuition):**
- Identity: `f | transform(id) == f`
- Composition: `f | transform(g) | transform(h) == f | transform(h∘g)`

> Many things beyond containers are functors (e.g., `std::optional`).

---

### **Example: `std::optional` as a functor**

```cpp
template <typename T1, typename F>
auto transform(const std::optional<T1>& opt, F f)
    -> decltype(std::make_optional(f(opt.value())))
{
    if (opt) return std::make_optional(f(*opt));
    return {};
}
```

Use: lift pure functions to run **only if** a value exists; otherwise propagate emptiness.

---

### **Ranges note**
Range views (`views::transform`) act like functor `transform`, but always return a **range**, not the same wrapper type (so you may need explicit conversions when mixing with `optional`).