SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(include)

set(IRT_PROJECT_SRC
    src/irt_serial_port.cpp
    src/trx_subnetwork.cpp
)

ADD_EXECUTABLE(run_trx_subnetwork ${IRT_PROJECT_SRC} src/run_trx_subnetwork.cpp)
TARGET_LINK_LIBRARIES(run_trx_subnetwork libserial_static)

