find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

find_package(gRPC CONFIG QUIET)
if(gRPC_FOUND)
  include_directories(${gRPC_INCLUDE_DIRS})
  set(CARLA_GRPC_LIBRARIES gRPC::grpc gRPC::grpc++)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GRPCPP REQUIRED grpc++)
  include_directories(${GRPCPP_INCLUDE_DIRS})
  link_directories(${GRPCPP_LIBRARY_DIRS})
  set(CARLA_GRPC_LIBRARIES ${GRPCPP_LIBRARIES})
endif()

set(source_files
    model/OpenCDAClient.cc
    proto/carla.grpc.pb.cc
    proto/carla.pb.cc
)

set(header_files
    model/OpenCDAClient.h
    proto/carla.grpc.pb.h
    proto/carla.pb.h
)

set(test_sources
    # Add test sources if any
)

build_lib(
  LIBNAME carla
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${Protobuf_LIBRARIES}
    ${CARLA_GRPC_LIBRARIES}
    ${libcore}
    ${libmobility}
    ${libinternet}
  TEST_SOURCES ${test_sources}
)
