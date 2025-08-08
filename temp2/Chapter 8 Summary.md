### **Key Points**
- **Persistent** (immutable) structures share nodes between versions to make copies **cheap**.
- Immutable singly linked lists are great for **stack-like** ops (prepend/head/tail O(1)); middle/end edits are **O(n)**.
- Memory mgmt: use **`std::shared_ptr`**; flatten destructor chains to avoid recursion depth issues.
- **Bitmapped Vector Trie** gives vector-like semantics with **O(1)** lookup and end-updates, plus cheap versioning.
- Not every op is fast: **insert/prepend/concat** remain **O(n)**, similar to `std::vector`.
