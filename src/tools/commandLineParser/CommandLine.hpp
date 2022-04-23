
namespace tools {

inline CommandLine::CommandLine(std::string description) :
    _description{std::move(description)},
    _arguments{}
{}

//----------------------------------------------------------------------------------------------------------------------

inline CommandLine::~CommandLine() = default;

//----------------------------------------------------------------------------------------------------------------------

inline void CommandLine::addArgument(std::vector<std::string> const& flags,
                                     Value const& value,
                                     std::string const& help,
                                     bool required)
{
    _arguments.emplace_back(Argument{flags, value, required, false, help});
}

} // namespace tools
