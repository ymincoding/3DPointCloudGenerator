
#ifndef SS22_PROJECT_TEST_PLYFILEWRITER_H
#define SS22_PROJECT_TEST_PLYFILEWRITER_H

#include <vector>

#include "FileWriter.h"

namespace io{

class PlyFileWriter : public FileWriter
{
public:
    PlyFileWriter(int numVertex, std::vector<std::string> properties);
    ~PlyFileWriter() override;

    PlyFileWriter() = delete;
    PlyFileWriter(PlyFileWriter const& other) = delete;
    PlyFileWriter& operator=(PlyFileWriter const& other) = delete;

    void setInputData(std::vector<std::string> const& input);
protected:
    //=========================================================================
    //                               METHODS
    //=========================================================================
    void doWrite() override;

private:
    //=========================================================================
    //                              ATTRIBUTES
    //=========================================================================
    std::string _headerInfo;
    std::vector<std::string> _inputData;
    int _numVertex;
};

} // namespace io

#endif //SS22_PROJECT_TEST_PLYFILEWRITER_H
