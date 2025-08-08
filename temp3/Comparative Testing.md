### **Golden oracle by comparison**
When building a new structure/algorithm, compare against a **trusted implementation**.

**BVT vs std::vector**
- Convert between them
- After each op (`push_back`, `pop_back`, `update`, …) assert **equal contents**

```cpp
for (auto const& xs : generate_random_vectors()){
    BVT b(xs);
    assert(equal(xs, b));
    auto xs2 = xs; xs2.push_back(42);
    assert(equal(xs2, b.push_back(42)));
    if(!xs.empty()){
        xs2 = xs; xs2.pop_back();
        assert(equal(xs2, b.pop_back()));
    }
}
```
Use this alongside property tests for broader coverage.
