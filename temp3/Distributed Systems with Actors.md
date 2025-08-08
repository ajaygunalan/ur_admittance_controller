### **Scale out without changing logic**
- Actors don’t share memory or time; they send **messages**.
- Delivery guarantees: per-actor **FIFO** (messages processed in send order).

**Porting paths**:
1) Single-threaded → Multithreaded: introduce **per-actor queues** and `.then`-style dispatch; keep the pipeline the same.
2) Multithreaded → Distributed: make messages **serializable**; deliver across the network; pipeline still unchanged.

Libraries to look at:
- **CAF** — C++ Actor Framework (lightweight actors, network transparency)
- **SObjectizer** — alternative with strong performance (no built-in distribution)
