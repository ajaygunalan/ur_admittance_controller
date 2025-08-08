### **Immutable Lists: Why they matter**
- Keep **past versions** cheap by **sharing nodes** across versions.
- Prepend/remove-head without copying the rest.
- Changes at the **end or middle** require copying the prefix up to the change.

---

### **Prepend (O(1))**
Create a new head that points to the old list.

```
l1:  A → B → C → D
l2 = prepend(E, l1)

l2:  E → A → B → C → D      // shares tail with l1
```

### **Tail (remove first, O(1))**
Reuse the old list’s second node as the new head.

```
l1:  A → B → C → D
l3 = tail(l1)

l3:  B → C → D              // shares tail with l1
```

---

### **Append / Insert (O(n))**
You **can’t** mutate the old tail pointer without affecting other versions.
So you must **copy** the path up to the insertion point.

```
append(E, l1)
copy: A,B,C,D  →  A → B → C → D → E
```

### **Complexities**
- O(1): get first, prepend, remove first
- O(n): get last, nth, append, insert at position, concatenate
