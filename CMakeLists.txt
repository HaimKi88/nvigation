cmake_minimum_required(VERSION 3.10)
project(MyOpenCVApp)  # Use a more descriptive name

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Optional: Make OpenCV_DIR overrideable from command line
if(NOT OpenCV_DIR)
    set(OpenCV_DIR "C:/opencv/build")
endif()

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} 
    navigation.cpp
    )
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
