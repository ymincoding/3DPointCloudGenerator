
#ifndef SS22_PROJECT_TEST_COMMANDLINE_H
#define SS22_PROJECT_TEST_COMMANDLINE_H

#include <string>
#include <variant>
#include <vector>

namespace tools {

/**
 * @brief Command line parser
 *
 * Adapted and extended from http://schneegans.github.io/tutorials/2019/08/06/commandline.
 * Many thanks to the original author, Simon Schneegans.
 *
 * This class is a simple and effective class to parse command line arguments.
 * For each possible argument it stores a pointer to a variable. When the
 * corresponding argument is set on the command line (given to the parse()
 * method) the variable is set to the given value. If the option is not set,
 * the variable is not touched. Hence it should be initialized to a default
 * state.
 * For each argument, several names (aliases) can be defined. Thus, the same
 * boolean could be set via '--help' or '-h'. While not required, it is a good
 * practice to precede the argument names with either '--' or '-'. Except for
 * booleans, a value is expected to be given. Booleans are set to 'true' if no
 * value is provided (that means they can be used as simple flags as in the
 * '--help' case). Values can be given in two ways: Either the option name and
 * the value should be separated by a space or by a '='. Here are some valid
 * examples:
 * --string="Foo Bar"
 * --string "Foo Bar"
 * --help
 * --help=false
 * --help true
 */
class CommandLine
{
public:
    /**
     * These are the possible variables the options may point to. Bool and
     * std::string are handled in a special way, all other values are parsed
     * with a std::stringstream. This std::variant can be easily extended if
     * the stream operator>> is overloaded. If not, you have to add a special
     * case to the parse() method.
     */
    using Value = std::variant<std::int32_t*,
                               double*,
                               float*,
                               bool*,
                               std::string*>;

    // The description is printed as part of the help message.
    explicit CommandLine(std::string description);
    ~CommandLine();

    CommandLine(CommandLine const& other) = delete;
    CommandLine& operator=(CommandLine const& other) = delete;

    /**
     * @brief Adds a command line argument
     *
     * A typical call would be like this:
     * bool printHelp = false;
     * cmd.addArgument({"--help", "-h"}, &printHelp, "Print this help message");
     * Then, after parse() has been called, printHelp will be true if the user
     * provided the flag.
     *
     * @param[in] flags The alias(es) identifying the argument
     * @param[in] value One of the supported types
     * @param[in] help Help message
     * @param[in] required Flag to indicate required arguments
     */
    void addArgument(std::vector<std::string> const& flags,
                     Value const& value,
                     std::string const& help,
                     bool required = false);

    /**
     * @brief Prints the description given to the constructor and the help for each option
     *
     * @param os[out] Target stream
     */
    void printHelp(std::ostream& os) const;

    /**
     * @brief Parses the command line
     *
     * The command line arguments are traversed from start to end. That means,
     * if an option is set multiple times, the last will be the one which is
     * finally used.
     * An unknown flag will cause a warning.
     * A missing value (except for boolean types) will cause a fatal error.
     */
    void parse(int argc, char* argv[]);

    /**
     * @brief Sets the maximum line width for printHelp()
     *
     * @param[in] width Maximum width
     */
    static void setMaxLineWidth(std::size_t width);

private:
    //=========================================================================
    //                                TYPES
    //=========================================================================
    struct Argument
    {
        std::vector<std::string> flags;
        Value value;
        bool required;
        bool provided;
        std::string caption;
    };

    //=========================================================================
    //                               METHODS
    //=========================================================================
    std::pair<Argument*, bool> contains(std::string const& flag);
    bool needHelp();
    void ensureRequiredArguments() const;
    static void logWarning(std::string const& message);
    static void logFatal(std::string const& message);
    static void logArgumentNotFound(std::string const& argName);

    //=========================================================================
    //                              ATTRIBUTES
    //=========================================================================
    std::string _description;
    std::vector<Argument> _arguments;
    static std::size_t _maxLineWidth;
};

} // namespace tools

#include "CommandLine.hpp"

#endif //SS22_PROJECT_TEST_COMMANDLINE_H
