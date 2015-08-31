#include <boost/test/unit_test.hpp>
#include <lidar_sick_lms1xx/Dummy.hpp>

using namespace lidar_sick_lms1xx;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    lidar_sick_lms1xx::DummyClass dummy;
    dummy.welcome();
}
