### **Why ADTs?**
- Model only **valid program states** and make illegal states **unrepresentable**.
- Build types using two ops:
  - **Product** (AND): hold values of **both** types (structs, `std::pair`, `std::tuple`).
  - **Sum** (OR): hold a value of **one of** several types (`enum`, `std::variant`).

---

### **Product Types (struct / tuple)**
```cpp
struct state_t {
    bool started;
    bool finished;
    unsigned count;
    socket_t web_page;
};
```
- Easy but permits **nonsense combos** (e.g., `finished && !started`).

**Tuple tip:** great for **local** packing and lexicographic comparisons, but avoid as **public API** because fields are **unnamed**.

---

### **Sum Types (enum / variant)**
- Encode mutually exclusive states:
  - `init`
  - `running { count, web_page }`
  - `finished { count }`

> Sum types cut the state-space down to **only the sensible possibilities**.
