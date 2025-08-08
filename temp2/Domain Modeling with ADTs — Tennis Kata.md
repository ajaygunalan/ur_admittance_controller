### **Goal:** encode tennis scoring without illegal states
States:
- **normal_scoring**: both players in {0,15,30}
- **forty_scoring**: one player at 40, other in {0,15,30}
- **deuce**
- **advantage {player}**
- *(optionally)* **game_over {winner}**

```cpp
enum class points { love, fifteen, thirty };
enum class player { p1, p2 };

struct normal_scoring { points p1, p2; };
struct forty_scoring  { player leading; points other; };
struct deuce{};
struct advantage { player who; };

using game_state = std::variant<normal_scoring, forty_scoring, deuce, advantage>;
```
> Illegal combos like (40,40) are **impossible** by construction.

**Transition idea:** `point_for(player)` visits the variant and returns the **next** state.

