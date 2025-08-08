### **Pop Back (remove last)**
Cases mirror append:
1) Last leaf has >1 element → copy path + leaf; drop last.
2) Last leaf has 1 element → copy path; **prune** empty leaf and any now-empty ancestors.
3) Root becomes a single-child node → **shrink depth** by promoting child to root.

### **Other operations**
- **Insert / Prepend / Concatenate** → require shifting or reblocking many leaves ⇒ **O(n)** (same asymptotics as `std::vector`).

### **Performance picture**
- O(1): index access, push_back, pop_back, update-at-index (bounded by small depth)
- O(n): insert/prepend/concat
- Slightly worse locality than `std::vector` (multiple blocks), but **copying versions is cheap** via structural sharing.
