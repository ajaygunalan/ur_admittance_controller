### **Write properties, not examples**
Test general **laws** that must always hold.

**reverse**
- `xs == reverse(reverse(xs))`
- `size(xs) == size(reverse(xs))`
- `front(xs) == back(reverse(xs))` and vice‑versa

**sort**
- Preserves size
- `min(xs) == front(sort(xs))`, `max(xs) == back(sort(xs))`
- `is_sorted(sort(xs))`
- `sort(reverse(xs)) == sort(xs)`

Generate many random collections and assert these properties.
