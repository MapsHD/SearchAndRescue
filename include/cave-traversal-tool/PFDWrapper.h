#pragma once
#include <string>

// Returns false when the dialog is cancelled.
bool PFDOpenFile(const std::string& title, const std::string& filter_description, const std::string& filter, std::string& out);
bool PFDSaveFile(const std::string& title, const std::string& default_path, const std::string& filter_description, const std::string& filter, std::string& out);
