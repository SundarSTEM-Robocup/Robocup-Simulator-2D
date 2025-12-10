file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(Pathway ${SOURCES} ${HEADERS})
target_link_libraries(Pathway PUBLIC UtilsLib config eigen ${GLOBAL_LIBS} VisualsLib)
target_include_directories(Pathway PUBLIC include)


