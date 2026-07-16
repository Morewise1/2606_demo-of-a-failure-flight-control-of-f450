#pragma once

#include <csignal>

namespace amp::common {

inline volatile std::sig_atomic_t g_stop_requested = 0;

inline void handle_stop_signal(int) {
    g_stop_requested = 1;
}

inline void install_stop_handlers() {
    g_stop_requested = 0;
    std::signal(SIGINT, handle_stop_signal);
#ifdef SIGTERM
    std::signal(SIGTERM, handle_stop_signal);
#endif
}

inline bool stop_requested() {
    return g_stop_requested != 0;
}

inline void request_stop() {
    g_stop_requested = 1;
}

}  // namespace amp::common
