
#ifndef SS22_PROJECT_TEST_FILEWRITER_H
#define SS22_PROJECT_TEST_FILEWRITER_H

#include "FileIO.h"

namespace io {

class FileWriter : public FileIO
{
public:
    FileWriter();
    ~FileWriter() override;

    void write(std::string const& filename);

protected:
    //=========================================================================
    //                               METHODS
    //=========================================================================
    void open(std::string const& filename);

    virtual void doWrite() = 0;
};

} // namespace io

#include "FileWriter.hpp"

#endif //SS22_PROJECT_TEST_FILEWRITER_H
