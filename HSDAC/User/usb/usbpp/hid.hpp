#pragma once
#include "usb.hpp"
#include <cstddef>
#include <cstdint>

struct HID_Descriptor_InitPack {
    uint16_t bcd_hid;
    uint8_t country_code;
};
struct HID_DescriptorLengthDesc {
    uint8_t type;
    uint16_t length;
};
// 1. one @HID_Descriptor_InitPack
// 2. any @HID_DescriptorLengthDesc
template<size_t N>
struct HID_Descriptor {
    static constexpr size_t len = 6 + 3 * N;
    CharArray<len> char_array {
        len,
        0x21
    };

    constexpr HID_Descriptor(
        HID_Descriptor_InitPack pack,
        std::array<HID_DescriptorLengthDesc, N> descs
    ) {
        char_array[2] = pack.bcd_hid & 0xff;
        char_array[3] = pack.bcd_hid >> 8;
        char_array[4] = pack.country_code;
        char_array[5] = N;
        for (size_t i = 0; i < N; ++i) {
            size_t offset = 6 + i * 3;
            char_array[offset] = descs[i].type;
            char_array[offset + 1] = descs[i].length & 0xff;
            char_array[offset + 2] = descs[i].length >> 8;
        }
    }
};

// 1. one @InterfaceInitPackClassed
// 2. one @HID_Descriptor
// 3. any @Endpoint
template<class... DESCS>
struct HID_Interface : public Interface<DESCS...> {
    constexpr HID_Interface(
        InterfaceInitPackClassed pack,
        const DESCS&... descs
    ) : Interface<DESCS...>(
        InterfaceInitPack{
            .interface_no = pack.interface_no,
            .alter = pack.alter,
            .class_ = 0x03,
            .subclass = 0x00,
            .protocol = 0x00,
            .str_id = pack.str_id
        },
        descs...
    ) {}
};

enum class HIDBootProtocol {
    None = 0,
    Keyboard,
    Mouse
};

struct HID_BOOT_Interface_InitPack {
    uint8_t interface_no;
    uint8_t alter;
    uint8_t str_id;
    HIDBootProtocol protocol;
};
// 1. one @HID_BOOT_InterfaceInitPack
// 2. one @HID_Descriptor
// 3. any @Endpoint
template<class... DESCS>
struct HID_BOOT_Interface : public Interface<DESCS...> {
    constexpr HID_BOOT_Interface(
        HID_BOOT_Interface_InitPack pack,
        const DESCS&... descs
    ) : Interface<DESCS...>(
        InterfaceInitPack{
            .interface_no = pack.interface_no,
            .alter = pack.alter,
            .class_ = 0x03,
            .subclass = 0x01,
            .protocol = static_cast<uint8_t>(pack.protocol),
            .str_id = pack.str_id
        },
        descs...
    ) {}
};
