### **When state helps**
- `join` already keeps **mutable** list of sources to extend lifetimes.
- Some behaviors need explicit state (e.g., **throttling** a chatty client).

**Idea:** Accept single message from a client, then ignore that client for `Δt`.
- Map client → `next_allowed_time` (mono-clock).
- On each message: if `now < next_allowed_time[client]` → drop; else accept and set next time.

**Why safe without locks?**
- Actors are **isolated** and process messages **serially** → no data races inside an actor.
- In a multithreaded or distributed variant, messages are still delivered to a single-threaded actor loop.
