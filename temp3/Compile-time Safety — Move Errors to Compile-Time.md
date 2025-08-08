### **Idea**
Push as many bugs as possible from **runtime** to **compile time** with types and zero‑cost abstractions.

- Encode **units** and invariants in types to block nonsense states.
- Favor APIs that make **illegal states unrepresentable**.

**Example — distance types with units**
```cpp
template <typename Rep, typename Ratio = std::ratio<1>>
class distance { /* ... */ };

template <typename R> using meters      = distance<R>;
template <typename R> using kilometers  = distance<R, std::kilo>;
template <typename R> using centimeters = distance<R, std::centi>;
template <typename R> using miles       = distance<R, std::ratio<1609>>;

// user-defined literals
constexpr kilometers<long double> operator"" _km(long double v){ return {v}; }
// ...
auto d = 42.0_km + 1.5_mi; // ❌ type mismatch until you add explicit conversions
```
**Payoff:** errors surface **at compile time** instead of blowing up later.
