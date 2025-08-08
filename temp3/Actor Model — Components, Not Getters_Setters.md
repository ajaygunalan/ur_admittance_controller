### **Shift the mindset**
- Don’t design objects as bags of data with getters/setters.
- Design **actors**: isolated components that **receive messages** and **emit messages**.
- No shared state between actors. Concurrency-friendly by construction.

---

### **Minimal typed actor interface**
```cpp
template <typename SourceMessage, typename Message>
class actor {
public:
    using value_type = Message;

    void process_message(SourceMessage&& message);

    template <typename EmitFn>
    void set_message_handler(EmitFn emit);

private:
    std::function<void(Message&&)> m_emit;
};
```

**Rules used in this chapter**:
- Each actor accepts **one** input message type and emits **one** output message type.
- Routing (who talks to whom) is handled by an **external controller** → actors stay composable.
- Async/sync delivery is an **infrastructure** concern; actor *design* doesn’t change.
