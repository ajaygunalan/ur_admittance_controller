### Filter View
- Acts like a collection containing **only elements that satisfy** a predicate
- Implemented with a proxy iterator that **skips** non-matching items

```cpp
auto girls = people | views::filter(is_female);
```

> Incrementing the iterator advances to the **next matching** element.

---

### Transform View
- Produces one value per source element by applying a **mapping**
- Only dereference is special (computes `f(*it)`)

```cpp
auto names = people | views::transform(name);
```

---

### Take View
- Truncates to the **first _n_** elements (even if source is larger)
```cpp
auto top3_names = people
                | views::filter(is_female)
                | views::transform(name)
                | views::take(3);
```

---

### Zip View (with infinite ints)
- Zip any range with indexes using `views::ints(start)`

```cpp
auto ranked = views::zip(movies, views::ints(1))   // (movie, index)
            | views::transform([](auto&& p){
                  return std::to_string(std::get<1>(p)) + " " + std::get<0>(p);
               })
            | views::take(10);
for (auto&& s : ranked) std::cout << s << "\n";
```

