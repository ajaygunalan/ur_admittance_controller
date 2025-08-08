### Why Ranges?
- STL algorithms take **iterator pairs**, which:
  - Don’t compose nicely (need temp containers)
  - Encourage **mutating** intermediates
  - Are easy to misuse (mismatched begin/end)

**Ranges** bundle begin/end into one **range** object so you can pass results directly into the next step.

---

### From Nesting to Piping
Classic composition (harder to read):
```cpp
auto names = transform(filter(people, is_female), name);
```

Pipe syntax (range-v3 style):
```cpp
using namespace ranges;
auto names = people
           | views::filter(is_female)
           | views::transform(name);
```

> Think of `|` as “send through this transformation” (like UNIX pipes).

---

### Zero-Copy Views
- `filter` and `transform` create **views** (lightweight adapters) over existing data.
- No intermediate vectors; operations are **lazy** and evaluated **on demand**.

```cpp
auto first3 = people
            | views::filter(is_female)
            | views::transform(name)
            | views::take(3);
std::vector<std::string> names = first3 | to<std::vector>(); // materialize
```

