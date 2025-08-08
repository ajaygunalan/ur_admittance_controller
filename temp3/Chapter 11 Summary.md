### **Key Points**
- Use `decltype` + `std::declval` and strip **cv/ref** to compute types like `contained_type_t`.
- Prefer container-provided **`value_type`** when available.
- `void_t` + SFINAE + `constexpr if` → detect features and steer compilation.
- Build a **generic currying** utility using `std::is_invocable_v`, `std::apply`, and `std::invoke`.
- Tiny **DSLs** are viable in C++: model an AST with templates and make the surface syntax pleasant.
