
#include "FileWriter.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace io {

void FileWriter::write(std::string const& filename)
{
    open(filename);
    doWrite();
}

//----------------------------------------------------------------------------------------------------------------------

void FileWriter::open(std::string const& filename)
{
    if (!fs::exists(fs::path(filename).parent_path()))
        fs::create_directories(fs::path(filename).parent_path());
    FileIO::openFile(filename, std::fstream::out);
}

} // namespace io
