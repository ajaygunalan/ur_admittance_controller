### **Vectors are great… until you need immutability**
- `std::vector` has contiguous storage → excellent locality & O(1) indexed access.
- But making a **modified copy** is O(n). We want **cheap versioning**.

### **Bitmapped Vector Trie (BVT)**
- Store elements in leaf blocks of **fixed capacity _m_** (power of 2; typically 32).
- Upper nodes hold **pointers to child blocks**. Depth grows as needed.
- Behaves like a vector but supports **structural sharing** across versions.

```
Level 0 (root): indexes children
Level 1: pointers to leaf blocks
Leaves: contiguous arrays of up to m elements
```

- Fill a leaf; allocate a new leaf. Fill the top-level index; allocate a new level.
