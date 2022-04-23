
#include "PlyFileWriter.h"

namespace io {

PlyFileWriter::PlyFileWriter(int numVertex, std::vector<std::string> properties) : FileWriter(),
                                                                                    _headerInfo{},
                                                                                    _inputData{},
                                                                                    _numVertex{numVertex}
{
    _headerInfo = "ply\n"
                  "format ascii 1.0\n"
                  "element vertex " + std::to_string(numVertex) + "\n";
    for (size_t index = 0; index < properties.size(); index++)
    {
        _headerInfo += "property " + properties[index] + "\n";
    }
    _headerInfo += "end_header\n";
}

//----------------------------------------------------------------------------------------------------------------------

PlyFileWriter::~PlyFileWriter() = default;

//----------------------------------------------------------------------------------------------------------------------

void PlyFileWriter::setInputData(std::vector<std::string> const& input)
{
    _inputData = input;
}

//----------------------------------------------------------------------------------------------------------------------

void PlyFileWriter::doWrite()
{
    this->_storage << _headerInfo;
    int count{0};

    for (auto& data : _inputData)
    {
        this->_storage << data;
        count++;

        if(count == _numVertex)
            break;
    }
}

//----------------------------------------------------------------------------------------------------------------------

} // namespace io