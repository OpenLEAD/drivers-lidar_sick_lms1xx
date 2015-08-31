rock_library(lidar_sick_lms1xx
    SOURCES Dummy.cpp
            Driver.cpp
    HEADERS Dummy.hpp
            Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(lidar_sick_lms1xx_test
        Main.cpp
        DEPS lidar_sick_lms1xx)
