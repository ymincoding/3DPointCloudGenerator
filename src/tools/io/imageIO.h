
#ifndef SS22_PROJECT_TEST_IMAGE_IO
#define SS22_PROJECT_TEST_IMAGE_IO

#include <iosfwd>

#include <opencv4/opencv2/core/mat.hpp>

namespace io {

cv::Mat loadImage(std::string const& filename);

cv::Mat loadDepthmap(std::string const& filename);

} // namespace io

#endif //SS22_PROJECT_TEST_IMAGE_IO
