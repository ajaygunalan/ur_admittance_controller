### **Vector as a Monad**
`transform` returns one output per input. `mbind` can return **0..N** outputs per input (think: `transform` + `join`).

```cpp
template <typename T, typename F>
auto mbind(const std::vector<T>& xs, F f) {
    auto chunks = xs | view::transform(f) | to_vector;
    return chunks | view::join | to_vector;
}
```

**Filter via `mbind`:**
```cpp
template <typename C, typename P>
auto filter(const C& xs, P pred) {
    return xs | mbind([=](auto e){
        return view::single(e) | view::take(pred(e) ? 1 : 0);
    });
}
```

---

### **Range/Monad Comprehensions**
Generate Pythagorean triples (`x≤y<z`, `x^2 + y^2 == z^2`):

```cpp
view::for_each(view::ints(1), [](int z){
    return view::for_each(view::ints(1, z), [=](int y){
        return view::for_each(view::ints(y, z), [=](int x){
            return yield_if(x*x + y*y == z*z, std::make_tuple(x,y,z));
        });
    });
});
```
This is sugar over nested `mbind` + `filter`.