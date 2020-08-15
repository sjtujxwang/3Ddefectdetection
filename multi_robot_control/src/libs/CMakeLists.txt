set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

######################################## TOOLS ##############################################

add_library(config SHARED config.cc)
add_library(dbscan SHARED dbscan.cc)
add_library(DefectDetect SHARED DefectDetect.cc)
add_library(gocator3100 SHARED gocator3100.cc)
add_library(gocator3200 SHARED gocator3200.cc)
add_library(pointcloud_helper SHARED pointcloud_helper.cc)
add_library(Utility SHARED Utility.cc)

######################################## install ##############################################
INSTALL(TARGETS config Utility
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION libstatic
)
