#pragma once
#include "usb.hpp"
#include <cstdint>

// --------------------------------------------------------------------------------
// UAC2
// --------------------------------------------------------------------------------

struct AudioFunctionInitPack {
    uint16_t bcd_adc;
    uint8_t catalog;
    uint8_t controls;
};
// 1. one @AudioFunctionInitPack
// 2. any AudioFunctions
template<class... DESCS>
struct AudioFunction {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + 9;
    CharArray<len> char_array {
        9,
        0x24,
        0x01
    };
    size_t begin = 9;

    constexpr AudioFunction(AudioFunctionInitPack pack, const DESCS&... desc) {
        char_array[3] = pack.bcd_adc & 0xff;
        char_array[4] = pack.bcd_adc >> 8;
        char_array[5] = pack.catalog;
        char_array[6] = len & 0xff;
        char_array[7] = len >> 8;
        char_array[8] = pack.controls;
        (AppendDesc(desc),...);
    }

    template<class DESC>
    constexpr void AppendDesc(const DESC& desc) {
        size_t desc_len = desc.len;
        for (size_t i = 0; i < desc_len ; ++i) {
            char_array[begin + i] = desc.char_array[i];
        }
        begin += desc_len;
    }
};

struct Clock {
    static constexpr size_t len = 8;
    CharArray<len> char_array {
        8,
        0x24,
        0x0a
    };

    constexpr Clock(uint8_t id, uint8_t attribute, uint8_t controls, uint8_t associate, uint8_t str_id) {
        char_array[3] = id;
        char_array[4] = attribute;
        char_array[5] = controls;
        char_array[6] = associate;
        char_array[7] = str_id;
    }
};

struct ChannelInitPack {
    uint8_t num_channel;
    uint32_t channel_config;
    uint8_t channel_str_id;
};

struct InputTerminal {
    static constexpr size_t len = 17;
    CharArray<len> char_array {
        17,
        0x24,
        0x02
    };

    constexpr InputTerminal(uint8_t id, uint16_t type, uint8_t associate, uint8_t clock_id, uint16_t control, uint8_t str_id, ChannelInitPack pack) {
        char_array[3] = id;
        char_array[4] = type & 0xff;
        char_array[5] = type >> 8;
        char_array[6] = associate;
        char_array[7] = clock_id;
        char_array[8] = pack.num_channel;
        char_array[9] = pack.channel_config & 0xff;
        char_array[10] = pack.channel_config >> 8;
        char_array[11] = pack.channel_config >> 16;
        char_array[12] = pack.channel_config >> 24;
        char_array[13] = control & 0xff;
        char_array[14] = control >> 8;
        char_array[15] = pack.channel_str_id;
        char_array[16] = str_id;
    }
};

struct FeatureUnitInitPack {
    uint8_t id;
    uint8_t source_id;
    uint8_t str_id;
};
// !!! this class have to manually set len in template
template<size_t N>
struct FeatureUnit {
    static constexpr size_t len = 6 + N * 4;
    CharArray<len> char_array {
        18,
        0x24,
        0x06
    };

    constexpr FeatureUnit(FeatureUnitInitPack pack, std::array<uint32_t, N> channel_features) {
        char_array[3] = pack.id;
        char_array[4] = pack.source_id;
        char_array[len - 1] = pack.str_id;
        for (size_t i = 0; i < N; ++i) {
            size_t offset = 5 + i * 4;
            uint32_t feature = channel_features[i];
            char_array[offset] = feature & 0xff;
            char_array[offset + 1] = feature >> 8;
            char_array[offset + 2] = feature >> 16;
            char_array[offset + 3] = feature >> 24;
        }
    }
};

struct OutputTerminal {
    static constexpr size_t len = 12;
    CharArray<len> char_array {
        12,
        0x24,
        0x03
    };

    constexpr OutputTerminal(uint8_t id, uint16_t type, uint8_t associate, uint8_t input_id, uint8_t clock_id, uint16_t controls, uint8_t str_id) {
        char_array[3] = id;
        char_array[4] = type & 0xff;
        char_array[5] = type >> 8;
        char_array[6] = associate;
        char_array[7] = input_id;
        char_array[8] = clock_id;
        char_array[9] = controls & 0xff;
        char_array[10] = controls >> 8;
        char_array[11] = str_id;
    }
};

// 1. one @InterfaceInitPack
// 2. one @AudioFunction
template<class... AUDIOFUNCTION>
struct AudioControlInterface : public Interface<AudioFunction<AUDIOFUNCTION...>> {
    constexpr AudioControlInterface(
        InterfaceInitPackClassed pack,
        const AudioFunction<AUDIOFUNCTION...>& function
    ) : Interface<AudioFunction<AUDIOFUNCTION...>>(
        InterfaceInitPack{
            .interface_no = pack.interface_no,
            .alter = pack.alter,
            .class_ = 1,
            .subclass = 1,
            .protocol = pack.protocol,
            .str_id = pack.str_id
        },
        function
    ) {}
};

struct AudioStreamFormat {
    static constexpr size_t len = 6;
    CharArray<len> char_array {
        6,
        0x24,
        0x02
    };

    constexpr AudioStreamFormat(uint8_t format_type, uint8_t subslotsize, uint8_t bits) {
        char_array[3] = format_type;
        char_array[4] = subslotsize;
        char_array[5] = bits;
    }
};

struct TerminalLink {
    static constexpr size_t len = 16;
    CharArray<len> char_array {
        len,
        0x24,
        1
    };

    constexpr TerminalLink(uint8_t link, uint8_t control, uint8_t format, uint32_t formats, ChannelInitPack pack) {
        char_array[3] = link;
        char_array[4] = control;
        char_array[5] = format;
        char_array[6] = formats & 0xff;
        char_array[7] = formats >> 8;
        char_array[8] = formats >> 16;
        char_array[9] = formats >> 24;
        char_array[10] = pack.num_channel;
        char_array[11] = pack.channel_config & 0xff;
        char_array[12] = pack.channel_config >> 8;
        char_array[13] = pack.channel_config >> 16;
        char_array[14] = pack.channel_config >> 24;
        char_array[15] = pack.channel_str_id;
    }
};

// this class contains 2 interfaces, one alter=0 and one alter=1
// 1. one @InterfaceInitPackClassed, alter will be ignored
// 2. one @TerminalLink
// 3. any @AudioStreamFormat
// 4. any @Endpoint
template<class... DESCS>
struct AudioStreamInterface : public IConfigCustom, public IInterfaceAssociationCustom {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + TerminalLink::len + 9 * 2;
    CharArray<len> char_array;
    size_t begin = 0;

    constexpr AudioStreamInterface(
        InterfaceInitPackClassed pack,
        TerminalLink link,
        const DESCS&... desc
    ) {
        Interface<> empty_interface{
            InterfaceInitPack{
                .interface_no = pack.interface_no,
                .alter = 0,
                .class_ = 1,
                .subclass = 2,
                .protocol = pack.protocol,
                .str_id = pack.str_id
            }
        };
        Interface<TerminalLink, DESCS...> interface{
            InterfaceInitPack{
                .interface_no = pack.interface_no,
                .alter = 1,
                .class_ = 1,
                .subclass = 2,
                .protocol = 0x20,
                .str_id = pack.str_id
            },
            link,
            desc...
        };
        for (size_t i = 0; i < empty_interface.len; ++i) {
            char_array[i] = empty_interface.char_array[i];
        }
        begin = empty_interface.len;
        for (size_t i = 0; i < interface.len; ++i) {
            char_array[begin + i] = interface.char_array[i];
        }
    }

    template<class... CONFIG_DESCS>
    constexpr void OnAddToConfig(Config<CONFIG_DESCS...>& config) const {
        config.char_array[IConfig::num_interface_offset]++;
    }

    template<class... OTHER_DESCS>
    constexpr void OnAddToInterfaceAssociation(InterfaceAssociation<OTHER_DESCS...>& association) const {
        if (association.char_array[IInterfaceAssociation::interface_count_offset] == 0) {
            association.char_array[IInterfaceAssociation::first_interface_offset] = char_array[IInterface::interface_no_offset];
        }
        association.char_array[IInterfaceAssociation::interface_count_offset]++;
    }
};

struct UAC2_InterfaceAssociation_InitPack {
    uint8_t str_id;
    uint8_t protocol;
};
// 1. one @UAC2_InterfaceAssociation_InitPack
// 2. one @AudioControlInterface
// 3. any @AudioStreamInterface
template<class... INTERFACE>
struct UAC2_InterfaceAssociation : public InterfaceAssociation<INTERFACE...> {
    constexpr UAC2_InterfaceAssociation(
        UAC2_InterfaceAssociation_InitPack pack,
        const INTERFACE&... interface
    ) : InterfaceAssociation<INTERFACE...>(
        InterfaceAssociationInitPack{
            .function_class = 1,
            .function_subclass = 0,
            .function_protocol = pack.protocol,
            .function_str_id = pack.str_id
        },
        interface...
    ) {}
};
