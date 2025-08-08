### **Inheritance (Open Sum)**
```cpp
struct state_t { int tag; virtual ~state_t() = default; };
struct init_t     : state_t { init_t(){ tag=0; } };
struct running_t  : state_t { running_t(){ tag=1; } unsigned m_count{}; socket_t m_web_page; };
struct finished_t : state_t { finished_t(unsigned c){ tag=2; } unsigned m_count; };

std::unique_ptr<state_t> st = std::make_unique<init_t>();
```
**Pros:** extensible (new subclasses).  
**Cons:** heap allocation, RTTI/tags, virtuals, nullptr hazards.

---

### **Closed Sum with `std::variant`**
```cpp
struct init_t{};
struct running_t{ unsigned m_count{}; socket_t m_web_page; };
struct finished_t{ unsigned m_count; };

class program_t {
    std::variant<init_t, running_t, finished_t> state = init_t{};

    void counting_finished() {
        auto* r = std::get_if<running_t>(&state);
        assert(r);
        state = finished_t{ r->m_count };
    }
};
```
**Pros:** value semantics, no heap/virtuals, type-safe access (`get`, `get_if`, `index`).  
**Cons:** closed set (must edit type to add a case).

---

### **State-local logic vs transitions**
- Put **state-specific behavior** inside the state (e.g., `running_t::count_words()`).
- Keep **transitions** (switching the variant) in the owner (`program_t`).

