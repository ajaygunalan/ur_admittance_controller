### **Key Points**
- ADTs (sum/product) help **ban invalid states**.
- Prefer `std::variant` for **closed** sums; inheritance only when you need **open** extension.
- Keep **state behavior** with the state; keep **transitions** with the owner.
- Use `std::optional` for “maybe” and `expected<T,E>` for “value or error with context”.
- Do pattern-style dispatch via `std::visit` + `overloaded`; reach for **Mach7** for richer patterns.
