### **Problem**
Member pointers and data members aren’t callable with `()` uniformly.

### **Solution**
- `std::invoke(callable, args...)` → works for free functions, functors, lambdas, **member functions**, **member data**.
- `std::apply(callable, tuple)`   → like `invoke` but takes a **tuple** of args.

Examples:
```cpp
std::invoke(std::less<>{}, 12, 14);
std::invoke(&person_t::name, martha);
std::apply(print_person, std::make_tuple(martha, std::ref(std::cout), person_t::name_only));
```
Curried uses `apply` because args are stored in a **tuple**.
