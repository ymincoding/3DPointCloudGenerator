
#include <iostream>
#include <vector>
#include <filesystem>

#include "tools/commandLineParser/CommandLine.h"
#include "tools/io/imageIO.h"
#include "tools/utils/utils.h"
#include "tools/io/PlyFileWriter.h"
#include "tools/core/MathUtils.h"

using std::cout;
using std::endl;
using std::string;

struct CmdLineArguments
{
    string argImageDir;
    string argDepthmapDir;
    string argOutputDir;
    bool argPrintHelp = false;
};
CmdLineArguments arguments;

//======================================================================================================================
//                                          FUNCTIONS DECLARATION
//======================================================================================================================
void addOptions(tools::CommandLine& cmdLineParser);
int run();
int main(int argc, char** argv);

//======================================================================================================================
//                                          FUNCTIONS IMPLEMENTATION
//======================================================================================================================
void addOptions(tools::CommandLine& cmdLineParser)
{
    cmdLineParser.addArgument({"-i", "--imageDir"},
                              &arguments.argImageDir,
                              "[REQUIRED] An image directory",
                              true);
    cmdLineParser.addArgument({"-d", "--depthmapDir"},
                              &arguments.argDepthmapDir,
                              "[REQUIRED] A depth map directory",
                              true);
    cmdLineParser.addArgument({"-o", "--outputDir"},
                              &arguments.argOutputDir,
                              "[REQUIRED] An output directory",
                              true);
    cmdLineParser.addArgument({"-h", "--help"}, &arguments.argPrintHelp, "[OPTIONAL] Print this help");
}

//----------------------------------------------------------------------------------------------------------------------

int run()
{
    using Filenames = std::vector<string>;
    Filenames KImgFileList{utils::listFilesInDirectory(arguments.argImageDir, true, ".jpg")};
    Filenames KDepthmapFileList{utils::listFilesInDirectory(arguments.argDepthmapDir, true, ".exr")};
    if (KImgFileList.size() != KDepthmapFileList.size())
    {
        cout << "ERROR: Number of images and depth maps should be the same." << endl;
        return -1;
    }
    const size_t KNumberOfData{KImgFileList.size()};

    // Set PLY properties
    std::vector<string> properties{"double x", "double y", "double z", "uchar red", "uchar green", "uchar blue"};

    for (size_t index = 0; index < KNumberOfData; index++)
    {
        cv::Mat KImage{io::loadImage(KImgFileList[index])};
        cv::Mat KDepthMap{io::loadDepthmap(KDepthmapFileList[index])};

        std::vector<string> vertexList;
        vertexList.resize(KImage.rows * KImage.cols);
        int vertexCount{0};
        for (int row = 0; row < KImage.rows; row++)
        {
            for (int col = 0; col < KImage.cols; col++)
            {
                // Get 3D Position of each pixel
                const core::EVector3 KUnitCartesianCoord{core::sphereMapCoordsToUnitCartesian(col,
                                                                                              row,
                                                                                              KImage.cols,
                                                                                              KImage.rows)};
                const float& KDepthValue{KDepthMap.at<float>(row, col)};
                if (KDepthValue < 0 || KDepthValue > 100)
                    continue;
                const auto backprojected3DPoints{KUnitCartesianCoord * KDepthValue};

                // Get RGB color of each pixel
                const cv::Vec3b& KPixelColor{KImage.at<cv::Vec3b>(row, col)};

                // Make one line of string
                string inputData{std::to_string(backprojected3DPoints[0]) + " "
                                 + std::to_string(backprojected3DPoints[1]) + " "
                                 + std::to_string(backprojected3DPoints[2]) + " "
                                 + std::to_string(KPixelColor[2]) + " "
                                 + std::to_string(KPixelColor[1]) + " "
                                 + std::to_string(KPixelColor[0]) + "\n"};
                vertexList[vertexCount] = inputData;
                vertexCount++;
            }
            cout << "Count: " << vertexCount << endl;
        }

        // open ply file and write header
        io::PlyFileWriter plyFileWriter(vertexCount, properties);
        plyFileWriter.setInputData(vertexList);
        plyFileWriter.write((std::filesystem::path(arguments.argOutputDir) / "output.ply").native());
    }

    return EXIT_SUCCESS;
}

//======================================================================================================================
//                                                  MAIN
//======================================================================================================================
int main(int argc, char** argv)
{
    tools::CommandLine cmdLineParser("SS22 Project Test Task");
    addOptions(cmdLineParser);
    cmdLineParser.parse(argc, argv);
    if (arguments.argPrintHelp || argc < 2)
    {
        cmdLineParser.printHelp(cout);
        return EXIT_SUCCESS;
    }

    if(run() == EXIT_SUCCESS)
        std::cout << "Colored 3D point cloud is generated!" << std::endl;

    return 0;
}
