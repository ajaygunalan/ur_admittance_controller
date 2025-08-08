### **Property: inverse problems**
If you can build a **generator** for inputs from a spec, test the function against **many random cases**.

**Lines example**
```cpp
std::string generate_test_case(int n){
    std::string r;
    for(int i=0;i<n;++i){ r += random_string_without_newline() + '\n'; }
    r += random_string_without_newline();
    return r;
}

for (int n = 0; n < MAX_NEWLINES; ++n){
    for (int k = 0; k < CASES_PER_N; ++k){
        auto s = generate_test_case(n);
        assert(count_lines(begin(s), end(s)) == n);
    }
}
```
**Tip:** Print and persist the **random seed** for reproducible failures.
