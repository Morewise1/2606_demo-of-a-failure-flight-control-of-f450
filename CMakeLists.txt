cmake_minimum_required(VERSION 3.16)
project(amp_fc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AMP_USE_SG2002_CSI_PORT "Use SG2002 CSI/FreeRTOS board-port symbols" OFF)

add_library(amp_fc_common INTERFACE)
target_include_directories(amp_fc_common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(rt_flight
    rt_core/main_rtos.cpp
    rt_core/Sense.cpp
    rt_core/Mpu6050I2cBackend.cpp
    rt_core/Hmc5883lI2cBackend.cpp
    rt_core/SensorDiagnostics.cpp
    rt_core/Receiver.cpp
    rt_core/RtosUartBackend.cpp
    rt_core/Estimator.cpp
    rt_core/Controller.cpp
    rt_core/Actuator.cpp
    rt_core/ActuatorDuoPwm.cpp
    rt_core/RtosPwmBackend.cpp
    rt_core/FlightRtNode.cpp
    rt_core/FlightRtNodeStage3.cpp
    rt_core/RtosCmdquIsrTransport.cpp
    rt_core/hal_impl.cpp
    rt_core/Sg2002CsiBoardPort.cpp
)
target_link_libraries(rt_flight PRIVATE amp_fc_common)
target_compile_definitions(rt_flight PRIVATE AMP_RTOS_FREERTOS=1)
if(AMP_USE_SG2002_CSI_PORT)
    target_compile_definitions(rt_flight PRIVATE AMP_USE_SG2002_CSI_PORT=1)
endif()

add_executable(linux_telemetry
    linux_core/main_linux.cpp
    linux_core/MailboxTransport.cpp
    linux_core/TelemetryNode.cpp
)
target_link_libraries(linux_telemetry PRIVATE amp_fc_common pthread m rt dl)
