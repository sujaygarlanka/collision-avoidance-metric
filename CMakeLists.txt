cmake_minimum_required(VERSION 3.10)
project(collision-avoidance-metric)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add OpenCV include directory
include_directories(
    /usr/include/eigen3/
    /usr/local/include/opencv4
)

# Specify the OpenCV static libraries
set(OpenCV_LIBS
    /usr/local/lib/libopencv_core.so
    /usr/local/lib/libopencv_imgproc.so
    /usr/local/lib/libopencv_imgcodecs.so
    /usr/local/lib/libopencv_highgui.so
    /usr/local/lib/libOpen3D.so
)

# Add additional dependencies
set(Extra_LIBS
    -lpng 
    -ljpeg 
    -lz 
    -ltiff 
    -lwebp 
    -pthread
)

# Specify your source files
add_executable(collision-avoidance-metric main.cpp)

# Link the libraries
target_link_libraries(collision-avoidance-metric ${OpenCV_LIBS} ${Extra_LIBS})
