### Views vs Actions
- **Views** (lazy): don’t own/mutate data — `views::filter`, `views::transform`, `views::take`, `views::group_by`, `views::reverse`, ...
- **Actions** (eager, mutating/materializing): operate **in place** or **produce containers** — `actions::sort`, `actions::unique`, `to<std::vector>()`, ...

```cpp
using namespace ranges;

auto words = read_text();                 // std::vector<std::string>
words |= actions::sort | actions::unique; // in-place on lvalue

auto deduped = read_text()                // temporary
             | actions::sort
             | actions::unique;          // reused in-place (no extra copy)
```

> Prefer **views** when you might not need everything; switch to **actions** when you want a concrete, reusable result.

---

### `views::unique` vs `actions::unique`
- `views::unique` → produces a view that **skips** consecutive duplicates
- `actions::unique` → **erases** consecutive duplicates **from the container**

