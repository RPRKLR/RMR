#include "map_reader.hpp"

MapReader::MapReader()
{
}

MapReader::~MapReader()
{
}

void MapReader::readMap(std::string filename, MapArea &maps)
{
    std::fstream file(filename);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line))
    {
        lines.push_back(line);
        std::cout << line << std::endl;
    }
}