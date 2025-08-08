### **Indexing by bit chunks**
Treat the index `i` as digits in base **m** (or bit-chunks since m is power-of-two).
Each chunk selects the **child** at that level; the last chunk indexes inside the leaf.

```
Example (m = 4) — find element 14:
14 (base 10) = 0b001110  →  [00][11][10]
root[00] → child[11] → leaf[10]
```

- Depth is small (m=32 ⇒ huge capacity in ≤5–7 levels for typical sizes).
- In practice, **lookup is O(1)** due to tiny, bounded depth.
