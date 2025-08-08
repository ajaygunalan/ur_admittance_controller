### **Why dynamic allocation + sharing?**
- A node can be part of **several lists** (multiple parents).
- When one list is destroyed, only nodes with **no remaining parents** should be freed.

---

### **Use `std::shared_ptr` for node ownership**
```cpp
template <typename T>
struct list {
    struct node {
        T value;
        std::shared_ptr<node> tail;
        ~node() {
            // Flatten recursive teardown to avoid deep destructor recursion
            auto next = std::move(tail);
            while (next) {
                if (!next.unique()) break;      // another owner exists
                std::shared_ptr<node> t;
                std::swap(t, next->tail);       // detach
                next.reset();                   // destroy leaf
                next = std::move(t);            // continue
            }
        }
    };
    std::shared_ptr<node> head_;
};
```

- Avoids **recursive destructor chains** that can overflow the stack.
- Note: the loop above isn’t magically thread-safe; multiple ops on the same nodes may need external synchronization.
