cmake_minimum_required(VERSION 3.10)
project(Navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "C:/opencv/install")
endif()

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
    main.cpp
    # scratch.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
