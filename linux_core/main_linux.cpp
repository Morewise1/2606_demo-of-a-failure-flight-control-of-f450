#include <iostream>
#include <memory>
#include <string>
#include <cstdlib>

#include "../common/SignalStop.hpp"
#include "TelemetryNode.hpp"

int main(int argc, char* argv[]) {
    amp::common::install_stop_handlers();
    std::string ini_path;
    if (argc > 1 && argv[1] != nullptr) {
        ini_path = argv[1];
    }
    std::unique_ptr<amp::linux_core::RcSource> rc_source = std::make_unique<amp::linux_core::NullRcSource>();
    if (std::getenv("AMP_USE_SIM_RC") != nullptr) {
        rc_source = std::make_unique<amp::linux_core::SimulatedRcSource>();
        std::cout << "[Linux] warning: simulated RC source enabled by AMP_USE_SIM_RC\n";
    }
    amp::linux_core::TelemetryNode node(std::move(rc_source), ini_path);
    const bool ok = node.run(0);
    std::cout << "[Linux] exit=" << (ok ? "ok" : "failed") << '\n';
    return ok ? 0 : 1;
}
