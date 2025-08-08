### **Append: copy only the path**
Cases:
1) **Space in last leaf** → copy path to that leaf, duplicate leaf, write value.
2) **No space in last leaf, but ancestor has room** → create a fresh leaf and link via copied ancestors.
3) **All full (including root)** → create a **new root**; hang the old root off it; then case (2).

```
v1  ... [leaf: ... Q R _ ]
v2 = append(S, v1)        // duplicate leaf & ancestors on the path

v2  ... [leaf: ... Q R S ]
```

### **Update at index: same as case (1)**
- Locate leaf for `i`, copy path + leaf, overwrite the single slot.
- Old trie is untouched; new trie shares everything except the path.
- Cost: **O(1)** given bounded depth.
