
#include "CommandLine.h"

#include <algorithm>
#include <iomanip>

#include <glog/logging.h>

using std::size_t;
using std::string;

namespace tools {

size_t CommandLine::_maxLineWidth{60};

void CommandLine::printHelp(std::ostream& os) const
{
    // Print the general description.
    os << _description << std::endl;

    // Find the argument with the longest combined flag length (in order to align the help messages).
    size_t maxFlagLength{0};
    for (auto const& argument : _arguments)
    {
        size_t flagLength{0};
        for (auto const& flag : argument.flags)
            flagLength += flag.size() + 2;   // Plus comma and space
        maxFlagLength = std::max(maxFlagLength, flagLength);
    }

    // Now print each argument.
    for (auto const& argument : _arguments)
    {
        string flags;
        for (auto const& flag : argument.flags)
            flags += flag + ", ";

        // Remove last comma and space and add padding according to the longest flags to align the help messages.
        std::stringstream sstr;
        sstr << std::left << std::setw(maxFlagLength) << flags.substr(0, flags.size() - 2);

        // Print the help for each argument. This is a bit more involved
        // since we do line wrapping for long descriptions.
        size_t spacePos{0};
        size_t lineWidth{0};
        while (spacePos != string::npos)
        {
            size_t nextspacePos{argument.caption.find_first_of(' ', spacePos + 1)};
            sstr << argument.caption.substr(spacePos, nextspacePos - spacePos);
            lineWidth += nextspacePos - spacePos;
            spacePos = nextspacePos;

            if (lineWidth > _maxLineWidth)
            {
                os << sstr.str() << std::endl;
                sstr = std::stringstream();
                sstr << std::left << std::setw(maxFlagLength - 1) << " ";
                lineWidth = 0;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::parse(int argc, char** argv)
{
    // Skip the first argument (name of the program).
    int i{1};
    while (i < argc)
    {
        // First we have to identify wether the value is separated by a space or a '='.
        string flag{argv[i]};
        string value;
        bool valueIsSeparate{false};

        // If there is an '=' in the flag, the part after the '=' is actually the value.
        size_t equalPos = flag.find('=');
        if (equalPos != string::npos)
        {
            value = flag.substr(equalPos + 1);
            flag = flag.substr(0, equalPos);
        }
        // Else the following argument is the value.
        else if (i + 1 < argc)
        {
            value = argv[i + 1];
            valueIsSeparate = true;
        }

        if (flag[0] != '-')
            logFatal("Invalid argument identifier \"" + flag + "\"!");

        // Search for an argument with the provided flag.
        Argument* argPtr{nullptr};
        bool foundArgument{false};
        std::tie(argPtr, foundArgument) = contains(flag);

        if (foundArgument)
        {
            argPtr->provided = true;
            Value& argValue = argPtr->value;

            // In the case of booleans, there must not be a value present.
            // So if the value is neither 'true' nor 'false' it is considered to be the next argument.
            if (std::holds_alternative<bool*>(argValue))
            {
                if (!value.empty() && value != "true" && value != "false")
                    valueIsSeparate = false;
                *std::get<bool*>(argValue) = (value != "false");
            }
            // In all other cases there must be a value.
            else if (value.empty())
                logFatal("Failed to parse command line arguments: Missing value for argument \"" + flag + "\"!");
            // For a string, we take the entire value.
            else if (std::holds_alternative<string*>(argValue))
            {
                if (value[0] == '-')
                    logFatal("Failed to convert command line argument \"" + flag + "\": string arguments may " +
                    "not start with \"-\". Is \"" + value + "\" perhaps another argument?");
                *std::get<string*>(argValue) = value;
            }
            // In all other cases we use a std::stringstream to convert the value.
            else
            {
                bool success;
                auto convert = [&value, &success](auto&& arg)
                {
                    std::stringstream sstr(value);
                    sstr >> *arg;
                    success = !sstr.fail();
                };
                std::visit(convert, argValue);
                if (!success)
                    logFatal("Failed to convert command line argument \"" + flag + "\": \"" + value +
                    "\" is not valid for this type!");
            }
        }
        else
            logArgumentNotFound(flag);

        // Advance to the next flag.
        ++i;

        // If the value was separated, we have to advance our index once more.
        if (foundArgument && valueIsSeparate)
            ++i;
    }

    // Only ensure required arguments if help is not needed or the program was ran with no arguments (i == 1).
    if ((i == 1) || needHelp())
        return;

    ensureRequiredArguments();
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::setMaxLineWidth(size_t width)
{
    _maxLineWidth = width;
}

//----------------------------------------------------------------------------------------------------------------------

std::pair<CommandLine::Argument*, bool> CommandLine::contains(string const& flag)
{
    for (auto& argument : _arguments)
    {
        if (std::find(argument.flags.begin(), argument.flags.end(), flag) != std::end(argument.flags))
            return std::make_pair(&argument, true);
    }
    return std::make_pair(nullptr, false);
}

//----------------------------------------------------------------------------------------------------------------------

bool CommandLine::needHelp()
{
    Argument* argPtr{nullptr};
    bool foundArgument{false};
    std::tie(argPtr, foundArgument) = contains("-h");
    if (foundArgument && argPtr->provided)
        return true;

    std::tie(argPtr, foundArgument) = contains("--help");
    if (foundArgument && argPtr->provided)
        return true;

    return false;
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::ensureRequiredArguments() const
{
    for (auto const& argument : _arguments)
    {
        if (argument.required && !argument.provided)
        {
            string message{"Argument "};
            for (size_t i = 0; i < argument.flags.size(); ++i)
            {
                message += "\"";
                message += argument.flags[i];
                message += "\"";
                message += (i < argument.flags.size() - 1 ? ", " : "");
            }
            message += " is required but was not provided.";
            logFatal(message);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::logWarning(std::string const& message)
{
    LOG(WARNING) << message;
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::logFatal(std::string const& message)
{
    LOG(FATAL) << message;
}

//----------------------------------------------------------------------------------------------------------------------

void CommandLine::logArgumentNotFound(std::string const& argName)
{
    logWarning("Ignoring unknown command line argument \"" + argName + "\".");
}

} // namespace tools
