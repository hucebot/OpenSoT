#ifndef __OPENSOT_RESOURCES_UTILS_H__
#define __OPENSOT_RESOURCES_UTILS_H__

#include <filesystem>
#include <optional>
#include <string>
#include <fstream>

namespace OpenSoT {

namespace resources_utils{

inline std::filesystem::path root()
{
    return std::filesystem::path(OPENSOT_RESOURCES_PATH);
}

inline std::optional<std::filesystem::path>
find(const std::string& filename)
{
    const auto base = root();

    for (auto const& entry :
         std::filesystem::recursive_directory_iterator(base))
    {
        if (entry.path().filename() == filename)
        {
            return entry.path();
        }
    }

    return std::nullopt;
}

std::string ReadFile(std::string path)
{
    std::ifstream t(path);
    std::stringstream buffer;
    buffer << t.rdbuf();
    return buffer.str();
}

}
}


#endif
