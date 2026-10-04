#include <Core/PFDWrapper.h>

#include <portable-file-dialogs.h>
#include <spdlog/spdlog.h>

#include <vector>

bool PFDOpenFile(const std::string& title, const std::string& filter_description, const std::string& filter, std::string& out)
{
    const std::vector<std::string> result = pfd::open_file(title, "", {filter_description, filter}).result();
    if (result.empty())
        return false;

    if (result.size() > 1)
        spdlog::warn("PFD result contains multiple entries - loading first ...");

    out = result.front();
    return true;
}

bool PFDSaveFile(const std::string& title, const std::string& default_path, const std::string& filter_description, const std::string& filter, std::string& out)
{
    const std::string result = pfd::save_file(title, default_path, {filter_description, filter}).result();
    if (result.empty())
        return false;

    out = result;
    return true;
}
