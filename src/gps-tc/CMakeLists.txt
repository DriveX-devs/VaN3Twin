set(source_files
    model/GeographicLib/utmups.c      
    model/GeographicLib/utmups_math.c
    model/gps-tc.cc
    helper/gps-tc-helper.cc)

set(header_files
    model/GeographicLib/utmups.h        
    model/GeographicLib/utmups_math.h
    model/gps-tc.h
    helper/gps-tc-helper.h)

set(test_sources
)

set_source_files_properties(model/GeographicLib/utmups.c      
    model/GeographicLib/utmups_math.c
	PROPERTIES SKIP_PRECOMPILE_HEADERS ON)

build_lib(
  LIBNAME gps-tc
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
	${libcore}
	${libmobility}
	${libvehicle-visualizer}
  ${libtraci}
  ${libsionna}
  TEST_SOURCES ${test_sources}
)
