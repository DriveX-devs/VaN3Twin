find_package(protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
link_libraries(${Protobuf_LIBRARIES})

find_package(gRPC CONFIG REQUIRED)
include_directories(${gRPC_INCLUDE_DIRS})
link_libraries(${gRPC_LIBRARIES})

link_libraries (protobuf::libprotobuf
    gRPC::grpc
    gRPC::grpc++)

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
#    ${gRPC}
    ${protobuf}
    ${libcore}
    ${libmobility}
    ${libinternet}
  TEST_SOURCES ${test_sources}
)

