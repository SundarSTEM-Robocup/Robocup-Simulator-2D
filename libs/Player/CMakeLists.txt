file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(PlayerManager ${SOURCES} ${HEADERS})
target_link_libraries(PlayerManager PUBLIC UtilsLib config eigen ${GLOBAL_LIBS} VisualsLib Traj Kin Pathway)
target_include_directories(PlayerManager PUBLIC include)



# add_subdirectory(test)
