### **Key Points**
- Think in **actors**: isolated components that only communicate via messages.
- Reactive streams can be treated as a **monad**: `transform`, `values`, `join`; plus `filter` for selection.
- Handle failures with **`expected` + `mtry` + `mbind`**; stop on first error.
- To reply to clients, carry the channel using **`with_client<T>`** and lift/unwrap at that level.
- Mutable state inside actors is okay (e.g., **throttling**) because mutation is **not shared**.
- Moving from single-threaded → multi-threaded → distributed changes the **delivery layer**, not the **pipeline design**.
