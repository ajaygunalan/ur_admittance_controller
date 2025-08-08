### **Why pure functions are test candy**
- A pure function depends **only** on its inputs and has **no side effects**.
- No “arrange complex state” ritual; just call it with arguments.

**Example**
```cpp
template <class Iter, class End>
int count_lines(Iter first, End last){
    return std::count(first, last, '\n');
}
```
Call it on **strings**, **ranges**, **streams**, or **linked lists** by passing iterators. No hidden state → fewer mocks, faster tests.
