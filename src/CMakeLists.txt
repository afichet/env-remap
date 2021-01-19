include_directories(${CMAKE_CURRENT_LIST_DIR}/../lib/tinyexr)

add_executable(envremap main.cpp)
    
install(TARGETS envremap RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})