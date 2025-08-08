### Key Points
- Ranges fix iterator-pair pain and make transformations **chainable**.
- **Views** are lazy, zero-copy adapters; **actions** are eager and mutating.
- Pipe syntax (`|`) keeps dataflow readable and linear.
- **Sentinels** simplify delimited sources and enable **infinite ranges** like `views::ints()`.
- Real-world flows (e.g., word counts) compose into short, loop-free programs.
