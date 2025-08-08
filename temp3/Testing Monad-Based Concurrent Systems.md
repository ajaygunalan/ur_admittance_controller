### **Turn async pipelines into sync tests**
Design your pipeline with **monadic** building blocks (`transform/map`, `mbind`, `join`, `filter`, `values`). Then swap the async stream for a **range** in tests.

**Reply channel test double**
```cpp
template <class T>
struct with_expected_reply{
    T value;
    std::string expected;
    void reply(const std::string& msg) const { REQUIRE(msg == expected); }
};
```
Lift your transforms/filters to work on `with_expected_reply<T>`, and drive the pipeline with a **vector** of inputs carrying their **expected replies**. You test **logic** deterministically while the production system uses sockets/async streams.
