### **Key Points**
- **Functors** lift pure functions over wrapped values; **monads** also flatten and compose functions that **return** wrapped values.
- `optional` / `expected` model failure without exceptions; `mbind` stops at first failure.
- **Try** pattern: adapt exception code to monadic pipes with `expected<..., exception_ptr>`.
- **Writer-style** (`with_log`) collects logs alongside values—purely.
- **Futures** form a continuation-style monad; use `.then` to chain async computations without blocking.
- **Kleisli composition** (`mcompose`) lets you compose monadic functions just like normal ones.
