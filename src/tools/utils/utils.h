
#ifndef SS22_PROJECT_TEST_UTILS_H
#define SS22_PROJECT_TEST_UTILS_H

#include <string>
#include <vector>

namespace utils {

/**
 * @brief Lists the files in the specified directory
 * @param[in] targetDir The target directory
 * @param[in] concatenate By default, the returned file names do not include \a targetDir. This flag enables
 *            concatenation of \a targetDir and the respective file names, i.e. full paths to the files are
 *            returned.
 * @param[in] extension Optional file extension used to select specific file types
 */
std::vector<std::string> listFilesInDirectory(std::string const& targetDir,
                                              bool concatenate = false,
                                              std::string const& extension = {});

} // namespace utils

#endif //SS22_PROJECT_TEST_UTILS_H
