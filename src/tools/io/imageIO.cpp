
#include "imageIO.h"

#include <filesystem>
#include <vector>

#include <glog/logging.h>
#include <opencv4/opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace io {

cv::Mat loadImage(std::string const& filename)
{
    CHECK(fs::exists(fs::path(filename))) << "File " << filename << " does not exist.";
    cv::Mat img = cv::imread(filename, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    return img;
}

//----------------------------------------------------------------------------------------------------------------------

cv::Mat loadDepthmap(std::string const& filename)
{
    cv::Mat depthmap = loadImage(filename);
    CHECK((depthmap.channels() == 1) || (depthmap.channels() == 3))
        << "Invalid number of channels for a depth map (" << depthmap.channels() << ")";
    if (depthmap.channels() == 3)
    {
        std::vector<cv::Mat> channels;
        cv::split(depthmap, channels);
        depthmap = channels[0].clone();
    }
    return depthmap;
}

} // namespace io
