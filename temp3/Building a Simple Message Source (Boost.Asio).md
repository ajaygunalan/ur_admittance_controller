### **Goal**
A source actor that accepts TCP connections and emits **lines** (strings) received from clients.

---

### **Service (source)**
```cpp
class service {
public:
    using value_type = std::string;

    explicit service(boost::asio::io_service& io, unsigned short port = 42042)
        : m_acceptor(io, tcp::endpoint(tcp::v4(), port)), m_socket(io) {}

    template <typename EmitFn>
    void set_message_handler(EmitFn emit) {
        m_emit = emit;
        do_accept();
    }

private:
    void do_accept() {
        m_acceptor.async_accept(m_socket, [this](const error_code& ec) {
            if (!ec) make_shared_session(std::move(m_socket), m_emit)->start();
            else std::cerr << ec.message() << std::endl;
            do_accept(); // accept next
        });
    }

    tcp::acceptor m_acceptor;
    tcp::socket   m_socket;
    std::function<void(std::string&&)> m_emit;
};
```

### **Per-connection session**
Keep itself alive via `enable_shared_from_this`; read-until-`\n`, emit lines.
```cpp
template <typename EmitFn>
class session : public std::enable_shared_from_this<session<EmitFn>> {
public:
    session(tcp::socket&& s, EmitFn emit) : m_socket(std::move(s)), m_emit(emit) {}

    void start() { do_read(); }

private:
    void do_read() {
        auto self = this->shared_from_this();
        boost::asio::async_read_until(m_socket, m_data, '\n',
            [this, self](const error_code& ec, std::size_t) {
                if (!ec) {
                    std::istream is(&m_data);
                    std::string line; std::getline(is, line);
                    m_emit(std::move(line));
                    do_read(); // next
                }
            });
    }
    tcp::socket m_socket;
    boost::asio::streambuf m_data;
    EmitFn m_emit;
};
```

> Users of `service` just see a **stream of strings**; session lifetimes are hidden inside.
