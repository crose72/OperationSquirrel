#include "datalog.h"

bool headingsWritten = false;
std::string dataFileName = "data";
std::string unusedDataFileName = "";
std::vector<std::vector<std::string>> data = {};

template <typename T>
std::string toString(const T& value) 
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

void logData(void)
{
    if (firstLoopAfterStartup == true)
    {
        unusedDataFileName = checkAndAppendFileName(dataFileName);
    }

    if (headingsWritten == false)
    {
        // Heading row
        data.push_back({"Time",
                        "Latitude",
                        "Longtitude",
                        "Altitude",
                        "Relative Altitude",
                        "GPS Vx",
                        "GPS Vy",
                        "GPS Vz",
                        "Heading",
                        "Roll",
                        "Pitch",
                        "Yaw",
                        "Rollspeed",
                        "Pitchspeed",
                        "YawSpeed",
                        "Accel X",
                        "Accel Y",
                        "Accel Z",
                        "Gyro X",
                        "Gyro Y",
                        "Gyro Z",
                        "Magnetic Field X",
                        "Magnetic Field Y",
                        "Magnetic Field Z"});
        writeToCSV(unusedDataFileName, data);
        headingsWritten = true;
    }

    // Clear data vector and write to next row
    data.clear();
    data.push_back({{toString(elapsedTimeSeconds),
                    toString(lat),
                    toString(lon),
                    toString(alt),
                    toString(relative_alt),
                    toString(vx),
                    toString(vy),
                    toString(vz),
                    toString(hdg),
                    toString(roll),
                    toString(pitch),
                    toString(yaw),
                    toString(rollspeed),
                    toString(pitchspeed),
                    toString(yawspeed),
                    toString(xacc),
                    toString(yacc),
                    toString(zacc),
                    toString(xgyro),
                    toString(ygyro),
                    toString(zgyro),
                    toString(xmag),
                    toString(ymag),
                    toString(zmag)}});
    writeToCSV(unusedDataFileName, data);
}

void writeToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data) 
{
    std::ofstream outFile(filename, std::ios_base::app); // Open in append mode
    if (!outFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file '" << filename << "'" << std::endl;
        return;
    }

    // Write data to the file, column-wise
    if (!data.empty()) 
    {
        size_t numRows = data.size();
        size_t numCols = data[0].size();

        for (size_t row = 0; row < numRows; ++row) 
        {
            for (size_t col = 0; col < numCols; ++col) 
            {
                outFile << data[row][col];
                if (col < numCols - 1) 
                {
                    outFile << ',';
                }
            }
            outFile << '\n';
        }
    }

    outFile.close();
    // std::cout << "File " << filename << " written successfully." << std::endl;
}

std::string checkAndAppendFileName(const std::string& filename) 
{
    int counter = 1;
    std::string newFilename = filename + ".csv"; // Add .csv extension initially
    std::ifstream file(newFilename);

    while (file.is_open() == true) 
    {
        file.close();
        // If the file exists, append the counter number and try again
        newFilename = filename + "_" + std::to_string(counter++) + ".csv";
        file.open(newFilename);
    }

    return newFilename;
}
