#include "MailboxTransport.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <iostream>
#include <string>
#include <utility>

#if defined(__linux__)
#if defined(__GNUC__)
#define AMP_FC_WEAK __attribute__((weak))
#else
#define AMP_FC_WEAK
#endif

extern "C" AMP_FC_WEAK bool amp_fc_allocate_ion_region(std::size_t size, void** virt, std::uintptr_t* phys, int* fd) {
    (void)size;
    (void)virt;
    (void)phys;
    (void)fd;
    return false;
}

extern "C" AMP_FC_WEAK void amp_fc_free_ion_region(void* virt, std::size_t size, int fd) {
    (void)virt;
    (void)size;
    (void)fd;
}

extern "C" AMP_FC_WEAK bool amp_fc_flush_ion_region(void* virt, std::size_t size, int fd) {
    (void)virt;
    (void)size;
    (void)fd;
    return true;
}
#endif

namespace amp::linux_core {

namespace {

#if defined(__linux__)
enum SystemCmdType {
    CmdquSend = 1,
    CmdquSendWait,
    CmdquSendWakeup,
};

constexpr auto kRtosCmdquSend = _IOW('r', CmdquSend, unsigned long);
constexpr auto kRtosCmdquSendWait = _IOW('r', CmdquSendWait, unsigned long);

struct valid_t {
    unsigned char linux_valid;
    unsigned char rtos_valid;
} __attribute__((packed));

union resv_t {
    valid_t valid;
    unsigned short mstime;
};

struct cmdqu_t {
    unsigned char ip_id;
    unsigned char cmd_id : 7;
    unsigned char block : 1;
    union resv_t resv;
    unsigned int param_ptr;
} __attribute__((packed)) __attribute__((aligned(0x8)));
#endif

constexpr std::uint16_t kInitSharedMemoryAckTimeoutMs = 100;
constexpr std::uint16_t kConfigAckTimeoutMs = 100;
constexpr std::uint16_t kArmAckTimeoutMs = 100;
constexpr std::uint16_t kEmergencyAckTimeoutMs = 50;

template <typename T>
void publishSlot(proto::SharedSlot<T>& slot, std::uint32_t seq, const T& payload) {
    slot.payload = payload;
    slot.payload_crc = proto::payload_crc(payload);
    std::atomic_thread_fence(std::memory_order_release);
    slot.seq = seq;
    std::atomic_thread_fence(std::memory_order_release);
}

}  // namespace

MailboxTransport::MailboxTransport(std::string device_path) : device_path_(std::move(device_path)) {}

MailboxTransport::~MailboxTransport() {
    close();
}

bool MailboxTransport::open() {
    close();
    if (!openSharedRegion()) {
        return false;
    }

#if defined(__linux__)
    if (shared_backend_ == SharedRegionBackend::Ion) {
        std::cout << "[Linux] shared cmd region via ION @ 0x" << std::hex << shared_phys_addr_ << std::dec << '\n';
    } else if (shared_backend_ == SharedRegionBackend::DevMem) {
        std::cout << "[Linux] shared cmd region via /dev/mem fallback @ 0x" << std::hex << shared_phys_addr_ << std::dec << '\n';
    } else if (shared_backend_ == SharedRegionBackend::Shm) {
        std::cout << "[Linux] shared cmd region via shm: " << shared_region_path_ << '\n';
    } else {
        std::cout << "[Linux] shared cmd region fallback (process local)\n";
    }
#endif

#if defined(__linux__)
    fd_ = ::open(device_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        fallback_mode_ = true;
        std::cout << "[Linux] mailbox unavailable, fallback mode enabled\n";
        return true;
    }
    fallback_mode_ = false;
    std::cout << "[Linux] mailbox opened: " << device_path_ << '\n';
    if (!notifySharedRegion()) {
        std::cerr << "[Linux] failed to notify RTOS shared memory address\n";
        close();
        return false;
    }
#else
    fallback_mode_ = true;
    std::cout << "[Linux] mailbox unavailable on this platform, fallback mode enabled\n";
#endif
    return true;
}

void MailboxTransport::close() {
#if defined(__linux__)
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#else
    fd_ = -1;
#endif
    fallback_mode_ = false;
    closeSharedRegion();
}

bool MailboxTransport::isLive() const {
    return fd_ >= 0 && !fallback_mode_;
}

bool MailboxTransport::sendSetpoint(const proto::CommandSetpoint& setpoint) {
    // Setpoint is high-rate data. Publish it through shared memory only; do
    // not enqueue a non-blocking cmdqu mailbox packet for every RC frame.
    return writeSetpointCache(setpoint);
}

bool MailboxTransport::sendArm(proto::ArmState state) {
    if (!writeArmCache(state)) {
        return false;
    }
    return sendCommand(static_cast<std::uint8_t>(proto::FlightCmdId::Arm), arm_seq_, true, kArmAckTimeoutMs);
}

bool MailboxTransport::sendHeartbeat(std::uint32_t seq) {
    // Heartbeat is periodic high-rate state. Shared memory is enough; mailbox
    // cmdqu is reserved for low-rate events that need ACK.
    return writeHeartbeatCache(seq);
}

bool MailboxTransport::sendEmergencyStop(std::uint32_t reason) {
    if (!writeEmergencyCache(reason)) {
        return false;
    }
    return sendCommand(
        static_cast<std::uint8_t>(proto::FlightCmdId::EmergencyStop),
        emergency_seq_,
        true,
        kEmergencyAckTimeoutMs);
}

bool MailboxTransport::sendConfig(const proto::FlightConfig& config) {
    if (!writeConfigCache(config)) {
        return false;
    }
    return sendCommand(static_cast<std::uint8_t>(proto::FlightCmdId::Config), config_seq_, true, kConfigAckTimeoutMs);
}

bool MailboxTransport::sendCommand(std::uint8_t cmd_id, std::uint32_t param, bool wait_ack, std::uint16_t timeout_ms) {
#if !defined(__linux__)
    (void)cmd_id;
    (void)param;
    (void)wait_ack;
    (void)timeout_ms;
    return true;
#else
    if (fallback_mode_) {
        return true;
    }
    if (fd_ < 0) {
        return false;
    }

    cmdqu_t cmd{};
    cmd.ip_id = 0;
    cmd.cmd_id = cmd_id;
    cmd.block = wait_ack ? 1 : 0;
    cmd.resv.mstime = timeout_ms;
    cmd.param_ptr = param;

    const unsigned long request = wait_ack ? kRtosCmdquSendWait : kRtosCmdquSend;
    const int ret = ::ioctl(fd_, request, &cmd);
    if (ret < 0) {
        std::cerr << "[Linux] ioctl failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
#endif
}

bool MailboxTransport::writeSetpointCache(const proto::CommandSetpoint& setpoint) {
    auto* region = shared_region_ ? shared_region_ : &fallback_region_;
    if (region->magic != proto::kSharedRegionMagic || region->version != proto::kSharedRegionVersion) {
        std::memset(region, 0, sizeof(*region));
        region->magic = proto::kSharedRegionMagic;
        region->version = proto::kSharedRegionVersion;
    }

    if (setpoint.seq == 0u) {
        ++setpoint_seq_;
    } else {
        setpoint_seq_ = setpoint.seq;
    }

    proto::CommandSetpoint payload = setpoint;
    payload.seq = setpoint_seq_;
    publishSlot(region->setpoint, setpoint_seq_, payload);
    return flushSharedRegion();
}

bool MailboxTransport::writeArmCache(proto::ArmState state) {
    auto* region = shared_region_ ? shared_region_ : &fallback_region_;
    if (region->magic != proto::kSharedRegionMagic || region->version != proto::kSharedRegionVersion) {
        return false;
    }
    ++arm_seq_;
    proto::CommandArm payload{};
    payload.state = static_cast<std::uint8_t>(state);
    publishSlot(region->arm, arm_seq_, payload);
    return flushSharedRegion();
}

bool MailboxTransport::writeHeartbeatCache(std::uint32_t seq) {
    auto* region = shared_region_ ? shared_region_ : &fallback_region_;
    if (region->magic != proto::kSharedRegionMagic || region->version != proto::kSharedRegionVersion) {
        return false;
    }
    heartbeat_seq_ = seq == 0u ? heartbeat_seq_ + 1u : seq;
    proto::Heartbeat payload{};
    payload.seq = heartbeat_seq_;
    payload.monotonic_us = monotonicNowUs();
    publishSlot(region->heartbeat, heartbeat_seq_, payload);
    return flushSharedRegion();
}

bool MailboxTransport::writeEmergencyCache(std::uint32_t reason) {
    auto* region = shared_region_ ? shared_region_ : &fallback_region_;
    if (region->magic != proto::kSharedRegionMagic || region->version != proto::kSharedRegionVersion) {
        return false;
    }
    ++emergency_seq_;
    proto::CommandEmergencyStop payload{};
    payload.reason = reason;
    publishSlot(region->emergency_stop, emergency_seq_, payload);
    return flushSharedRegion();
}

bool MailboxTransport::writeConfigCache(const proto::FlightConfig& config) {
    auto* region = shared_region_ ? shared_region_ : &fallback_region_;
    if (region->magic != proto::kSharedRegionMagic || region->version != proto::kSharedRegionVersion) {
        return false;
    }
    if (config.update_seq == 0u) {
        ++config_seq_;
    } else {
        config_seq_ = config.update_seq;
    }
    proto::FlightConfig payload = config;
    payload.update_seq = config_seq_;
    publishSlot(region->config, config_seq_, payload);
    return flushSharedRegion();
}

std::uint64_t MailboxTransport::monotonicNowUs() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

bool MailboxTransport::openSharedRegion() {
    std::memset(&fallback_region_, 0, sizeof(fallback_region_));
    fallback_region_.magic = proto::kSharedRegionMagic;
    fallback_region_.version = proto::kSharedRegionVersion;
    shared_region_ = &fallback_region_;
    shared_backend_ = SharedRegionBackend::Fallback;
    shared_phys_addr_ = 0u;
    shared_map_base_ = nullptr;
    shared_map_size_ = 0;
    ion_fd_ = -1;

#if defined(__linux__)
    if (!allocateSharedRegion()) {
        return false;
    }
#endif
    return initializeSharedRegion();
}

bool MailboxTransport::allocateSharedRegion() {
#if !defined(__linux__)
    return true;
#else
    if (const char* env_devmem = std::getenv("AMP_DEVMEM_PATH")) {
        if (std::string(env_devmem).size() > 0u) {
            devmem_path_ = env_devmem;
        }
    }
    if (const char* env_path = std::getenv("AMP_CMD_REGION_SHM")) {
        if (std::string(env_path).size() > 0u) {
            shared_region_path_ = env_path;
        }
    }

    const bool ion_disabled = [] {
        const char* value = std::getenv("AMP_DISABLE_ION");
        return value != nullptr && std::string(value) == "1";
    }();
    if (!ion_disabled && allocateIonSharedRegion()) {
        return true;
    }

    if (std::getenv("AMP_CMD_REGION_PHYS") != nullptr && allocateDevmemSharedRegion()) {
        return true;
    }

    return allocateShmSharedRegion();
#endif
}

bool MailboxTransport::allocateIonSharedRegion() {
#if !defined(__linux__)
    return false;
#else
    void* virt = nullptr;
    std::uintptr_t phys = 0u;
    int fd = -1;
    if (!amp_fc_allocate_ion_region(shared_region_size_, &virt, &phys, &fd) || virt == nullptr || phys == 0u) {
        return false;
    }

    shared_region_ = reinterpret_cast<proto::SharedCommandRegion*>(virt);
    shared_map_base_ = virt;
    shared_map_size_ = shared_region_size_;
    shared_phys_addr_ = phys;
    ion_fd_ = fd;
    shared_backend_ = SharedRegionBackend::Ion;
    return true;
#endif
}

bool MailboxTransport::allocateDevmemSharedRegion() {
#if !defined(__linux__)
    return false;
#else
    const char* env_phys = std::getenv("AMP_CMD_REGION_PHYS");
    if (env_phys == nullptr) {
        return false;
    }
    const auto parsed = std::strtoull(env_phys, nullptr, 0);
    if (parsed == 0ull) {
        return false;
    }

    shared_phys_addr_ = static_cast<std::uintptr_t>(parsed);
    devmem_fd_ = ::open(devmem_path_.c_str(), O_RDWR | O_SYNC);
    if (devmem_fd_ < 0) {
        return false;
    }

    const long page_size = ::sysconf(_SC_PAGESIZE);
    const std::uintptr_t page_mask = static_cast<std::uintptr_t>(page_size - 1);
    const std::uintptr_t page_base = shared_phys_addr_ & ~page_mask;
    const std::uintptr_t page_offset = shared_phys_addr_ - page_base;
    const std::size_t map_size = static_cast<std::size_t>(page_offset + shared_region_size_);
    void* mapped = mmap(nullptr, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, devmem_fd_, static_cast<off_t>(page_base));
    if (mapped == MAP_FAILED) {
        ::close(devmem_fd_);
        devmem_fd_ = -1;
        shared_phys_addr_ = 0u;
        return false;
    }

    shared_map_base_ = mapped;
    shared_map_size_ = map_size;
    shared_region_ = reinterpret_cast<proto::SharedCommandRegion*>(static_cast<std::uint8_t*>(mapped) + page_offset);
    shared_backend_ = SharedRegionBackend::DevMem;
    return true;
#endif
}

bool MailboxTransport::allocateShmSharedRegion() {
#if !defined(__linux__)
    return true;
#else
    shared_fd_ = ::open(shared_region_path_.c_str(), O_RDWR | O_CREAT, 0666);
    if (shared_fd_ < 0) {
        return true;
    }
    if (ftruncate(shared_fd_, static_cast<off_t>(sizeof(proto::SharedCommandRegion))) != 0) {
        ::close(shared_fd_);
        shared_fd_ = -1;
        return true;
    }
    void* mapped = mmap(nullptr, sizeof(proto::SharedCommandRegion), PROT_READ | PROT_WRITE, MAP_SHARED, shared_fd_, 0);
    if (mapped == MAP_FAILED) {
        ::close(shared_fd_);
        shared_fd_ = -1;
        return true;
    }

    shared_region_ = reinterpret_cast<proto::SharedCommandRegion*>(mapped);
    shared_map_base_ = mapped;
    shared_map_size_ = sizeof(proto::SharedCommandRegion);
    shared_backend_ = SharedRegionBackend::Shm;
    shared_phys_addr_ = 0u;
    return true;
#endif
}

bool MailboxTransport::initializeSharedRegion() {
    if (shared_region_ == nullptr) {
        return false;
    }
    if (shared_region_->magic != proto::kSharedRegionMagic || shared_region_->version != proto::kSharedRegionVersion) {
        std::memset(shared_region_, 0, sizeof(*shared_region_));
        shared_region_->magic = proto::kSharedRegionMagic;
        shared_region_->version = proto::kSharedRegionVersion;
    }
    return flushSharedRegion();
}

bool MailboxTransport::notifySharedRegion() {
    if (shared_phys_addr_ == 0u) {
        std::cerr << "[Linux] no physical shared-memory address; set up ION or AMP_CMD_REGION_PHYS fallback\n";
        return false;
    }
    if (shared_phys_addr_ > std::numeric_limits<std::uint32_t>::max()) {
        std::cerr << "[Linux] shared-memory physical address exceeds 32-bit cmdqu param_ptr\n";
        return false;
    }

    const auto phys32 = static_cast<std::uint32_t>(shared_phys_addr_);
    if (!sendCommand(
            static_cast<std::uint8_t>(proto::FlightCmdId::InitSharedMemory),
            phys32,
            true,
            kInitSharedMemoryAckTimeoutMs)) {
        return false;
    }

    std::cout << "[Linux] init shared memory notified phys=0x" << std::hex << shared_phys_addr_ << std::dec << '\n';
    return true;
}

bool MailboxTransport::flushSharedRegion() {
    std::atomic_thread_fence(std::memory_order_release);
#if defined(__linux__)
    if (shared_backend_ == SharedRegionBackend::Ion) {
        return amp_fc_flush_ion_region(shared_map_base_, shared_map_size_, ion_fd_);
    }
#endif
    return true;
}

void MailboxTransport::closeSharedRegion() {
#if defined(__linux__)
    if (shared_backend_ == SharedRegionBackend::Ion && shared_map_base_ != nullptr) {
        amp_fc_free_ion_region(shared_map_base_, shared_map_size_, ion_fd_);
    } else if (shared_map_base_ != nullptr && shared_map_size_ > 0u) {
        munmap(shared_map_base_, shared_map_size_);
    }
    shared_map_base_ = nullptr;
    shared_map_size_ = 0;
    if (ion_fd_ >= 0) {
        ion_fd_ = -1;
    }
    if (devmem_fd_ >= 0) {
        ::close(devmem_fd_);
        devmem_fd_ = -1;
    }
    if (shared_fd_ >= 0) {
        ::close(shared_fd_);
        shared_fd_ = -1;
    }
#endif
    shared_region_ = nullptr;
    shared_phys_addr_ = 0u;
    shared_backend_ = SharedRegionBackend::Fallback;
}

}  // namespace amp::linux_core
