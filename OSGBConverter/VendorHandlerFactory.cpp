// Copyright Johnlyon
//
// VendorHandlerFactory implementation
//

#include "IVendorHandler.h"
#include "ContextCaptureHandler.h"
#include "DJITerraHandler.h"
#include "PlatformCompat.h"

#include <cstring>

std::unique_ptr<IVendorHandler> VendorHandlerFactory::Create(const std::string& rootDir)
{
    // Check for DJI fingerprint: Block_* directories
    {
        auto entries = DirectoryLister::List(rootDir);
        for (auto& e : entries)
        {
            if (e.isDirectory && strncmp(e.name.c_str(), "Block_", 6) == 0)
                return std::make_unique<DJITerraHandler>();
        }
    }

    // Check for CC fingerprint: Data/ directory with Tile_* children
    std::string dataDir = rootDir + "/Data";
    if (DirectoryLister::IsDirectory(dataDir))
    {
        auto entries = DirectoryLister::List(dataDir);
        for (auto& e : entries)
        {
            if (e.isDirectory && strncmp(e.name.c_str(), "Tile_", 5) == 0)
                return std::make_unique<ContextCaptureHandler>();
        }
    }

    // Fallback: scan root for Tile_* or Block_* directly
    {
        auto entries = DirectoryLister::List(rootDir);
        for (auto& e : entries)
        {
            if (e.isDirectory && strncmp(e.name.c_str(), "Tile_", 5) == 0)
                return std::make_unique<ContextCaptureHandler>();
        }
    }

    return nullptr;
}

std::unique_ptr<IVendorHandler> VendorHandlerFactory::Create(DataVendor vendor)
{
    switch (vendor)
    {
    case DataVendor::ContextCapture:
        return std::make_unique<ContextCaptureHandler>();
    case DataVendor::DJITerra:
        return std::make_unique<DJITerraHandler>();
    default:
        return nullptr;
    }
}