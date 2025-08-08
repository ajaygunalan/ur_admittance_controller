### **Mach7 library (value + type patterns)**
Write patterns with `Match/Case` and deconstruct:
```cpp
void point_for(player who){
    Match(state)
    {
        Case(C<normal_scoring>())        // increment or to forty
        Case(C<forty_scoring>(who, _))   // winner (had 40)
        Case(C<forty_scoring>(_, 30))    // other had 30 → deuce
        Case(C<forty_scoring>())         // otherwise increment other
        Case(C<deuce>())                 // to advantage(who)
        Case(C<advantage>())             // win or back to deuce
    }
    EndMatch
}
```
- `_` is a wildcard.
- More expressive than `std::visit` for **value-based** guards.
- Trade-off: extra dependency and more **awkward syntax**.

