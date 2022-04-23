
#include "utils.h"

#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace utils {

std::vector<std::string> listFilesInDirectory(std::string const& targetDir,
                                              bool concatenate,
                                              std::string const& extension)
{
    // Lambda function 1
    auto func1 = [](fs::directory_iterator const& it)->bool
            { return fs::is_regular_file(it->status()); };

    // Lambda function 2
    auto func2 = [func1, extension](fs::directory_iterator const& it)->bool
            { return func1(it) && (it->path().extension() == extension); };

    std::function<bool(fs::directory_iterator const&)> myfunc = func1;
    if (!extension.empty())
        myfunc = func2;

    std::vector<std::string> imageNames;
    const fs::path KTargetDir{targetDir};
    for (fs::directory_iterator it{KTargetDir}; it != fs::directory_iterator(); ++it)
    {
        if (myfunc(it))
            imageNames.push_back(it->path().filename().native());
    }
    std::sort(imageNames.begin(), imageNames.end());

    if (concatenate)
    {
        for (auto& name : imageNames)
            name = fs::path(KTargetDir / name).native();
    }
    return imageNames;
}

} // namespace utils
