// Utility to locate a physical (or logical) Windows display by parsing EDID from registry
#pragma once

#include <optional>
#include <string>
#include <vector>
#include <cstdint>

struct DisplayEdidInfo {
    std::string device_instance_id;   // Full device instance path (eg. DISPLAY\\ABC1234#5&...)
    std::string monitor_name;         // Friendly name if discovered (from EDID descriptor 0xFC)
    uint16_t manufacturer_id = 0;     // PNP ID encoded as 16-bit (3 letters packed)
    uint16_t product_code = 0;        // Model number from EDID ("Model: 981" means product_code == 981)
    uint32_t serial_number = 0;       // 32-bit serial if provided (sometimes 0)
    uint8_t week_of_manufacture = 0;
    uint16_t year_of_manufacture = 0; // full year (e.g., 2024)
    // Parsed preferred timing (first detailed timing block) if present
    uint32_t preferred_width = 0;   // horizontal active pixels
    uint32_t preferred_height = 0;  // vertical active lines

    // Desktop coordinates (filled by helper when requested)
    int desktop_x = 0;
    int desktop_y = 0;
    int desktop_width = 0;
    int desktop_height = 0;
};

// Class offers a single static method for now; could be extended later.
class DisplayEdidFinder {
public:
    // Searches Windows registry for all display EDIDs and returns first match for product_code AND serial_number (if serial filter supplied).
    // If serial_number_filter is std::nullopt only product_code must match.
    static std::optional<DisplayEdidInfo> FindDisplayByEdid(uint16_t product_code_filter, std::optional<uint32_t> serial_number_filter = std::nullopt);

    // Enumerate all EDID entries (best effort). Provided mainly for debugging.
    static std::vector<DisplayEdidInfo> EnumerateAll();

    // Populate desktop position (monitor origin and current mode size) for a previously found EDID entry.
    // Returns true if successfully resolved under Windows.
    static bool PopulateDesktopCoordinates(DisplayEdidInfo &info);
};
