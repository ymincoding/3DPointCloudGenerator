
#ifndef SS22_PROJECT_TEST_FILEIO_H
#define SS22_PROJECT_TEST_FILEIO_H

#include <string>

#include <opencv2/core/persistence.hpp>
#include <fstream>

namespace io {

class FileIO
{
public:
    FileIO();
    virtual ~FileIO();

    FileIO(FileIO const& other) = delete;
    FileIO& operator=(FileIO const& other) = delete;

protected:
    //=========================================================================
    //                               METHODS
    //=========================================================================
    void openFile(std::string const& filename, std::ios_base::openmode mode);

    //=========================================================================
    //                              ATTRIBUTES
    //=========================================================================
    std::fstream _storage;
};

} // namespace io

#include "FileIO.hpp"

#endif //SS22_PROJECT_TEST_FILEIO_H
