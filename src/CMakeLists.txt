rock_library(laserscanner_sick_lms1xx
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(laserscanner_sick_lms1xx_bin Main.cpp
    DEPS laserscanner_sick_lms1xx)

