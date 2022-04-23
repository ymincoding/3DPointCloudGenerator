
#include "FileIO.h"

#include <filesystem>

#include <glog/logging.h>

namespace fs = std::filesystem;

namespace io {

void FileIO::openFile(std::string const& filename, std::ios_base::openmode mode)
{
    if (mode == std::fstream::in)
        CHECK(fs::exists(fs::path(filename))) << "File " << filename << " does not exist";

    _storage.open(filename, mode);
    CHECK(_storage.is_open()) << "Failed to open file " << filename;
}

} // namespace io
