cmake_minimum_required(VERSION 3.10)
project(collision-avoidance-metric)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add OpenCV include directory
include_directories(
    /usr/include/eigen3/
    /usr/include/opencv4
    /home/sujay/Code/collision-avoidance-metric/include
)

# Specify the OpenCV static libraries
set(OpenCV_LIBS
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
    /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
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
add_executable(collision-avoidance-metric tests/test.cpp src/GripperPaths.cpp)

# Link the libraries
target_link_libraries(collision-avoidance-metric ${OpenCV_LIBS} ${Extra_LIBS})
