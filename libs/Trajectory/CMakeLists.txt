file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(Traj ${SOURCES} ${HEADERS})
target_link_libraries(Traj PUBLIC UtilsLib config eigen ${GLOBAL_LIBS} VisualsLib Pathway)
target_include_directories(Traj PUBLIC include)



# add_subdirectory(test)
