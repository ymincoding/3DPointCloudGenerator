
namespace io {

inline FileIO::FileIO() :
    _storage{}
{}

//----------------------------------------------------------------------------------------------------------------------

inline FileIO::~FileIO()
{
    _storage.close();
}

} // namespace io
