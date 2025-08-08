### Goal
Print the **top N** most frequent words from input text.

### Pipeline
```cpp
using namespace ranges;

auto words = istream_view<std::string>(std::cin)
           | views::transform(string_to_lower)
           | views::transform(string_only_alnum)
           | views::remove_if(&std::string::empty)   // drop empties
           | to<std::vector>();                      // actions need a container

words |= actions::sort;

const auto results = words
    | views::group_by(std::equal_to<>())             // groups are ranges
    | views::transform([](auto const& group){
          auto b = begin(group), e = end(group);
          return std::pair{ distance(b, e), *b };    // (count, word)
      })
    | to<std::vector>()
    | actions::sort;                                  // ascending by count

for (auto const& p :
     results | views::reverse | views::take(N)) {
    std::cout << p.first << " " << p.second << "\n";
}
```

### Helpers
```cpp
std::string string_to_lower(std::string s) {
    return s | views::transform([](unsigned char c){ return std::tolower(c); })
             | to<std::string>;
}

std::string string_only_alnum(std::string s) {
    return s | views::filter([](unsigned char c){ return std::isalnum(c); })
             | to<std::string>;
}
```

> No explicit loops; small, **composable** steps; laziness where it counts, **eagerness** only when needed.
