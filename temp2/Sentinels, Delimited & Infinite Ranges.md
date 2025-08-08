### Sentinels (End Without an Iterator)
- End of range can be a **different type** (a “sentinel”), not necessarily an iterator
- Helps with **delimited** sources (e.g., stream reads until EOF, C-strings until `\0`)
- C++17 allows begin/end to have **different types** → sentinel-friendly

```cpp
auto sum = std::accumulate(std::istream_iterator<double>(std::cin),
                           std::istream_iterator<double>{}, 0.0);
```

With sentinels, equality checks can be **overloads**: (iterator, iterator) vs (iterator, sentinel), avoiding runtime type tests.

---

### Infinite Ranges
- Some ranges have **no end** (e.g., `views::ints()`)
- Compose with finite views (`take`, `zip`) to bound consumption

```cpp
auto first10 = views::ints(0) | views::take(10);          // 0..9
auto enumerate = views::zip(xs, views::ints(1));          // (x, 1..)
```

> Designing algorithms to work with infinite ranges often makes them **more generic** (they’ll work for any size).

