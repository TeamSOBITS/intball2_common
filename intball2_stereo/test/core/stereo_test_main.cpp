#include <gtest/gtest.h>
#include <opencv2/core.hpp>

int main(int argc, char **argv)
{
  // Serial test process: zero disables parallel backends rather than creating a
  // one-thread TBB arena. Keep LeakSanitizer enabled, including dependency code.
  cv::setNumThreads(0);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
