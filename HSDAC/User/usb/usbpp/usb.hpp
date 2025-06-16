#pragma once
#include <array>
#include <type_traits>
#include <cstdint>

// --------------------------------------------------------------------------------
// STANDARD USB
// --------------------------------------------------------------------------------

template<size_t N>
struct CharArray {
    static constexpr size_t desc_len = N;
    uint8_t desc[N]{};
    
    constexpr uint8_t& operator[](size_t i) {
        return desc[i];
    }

    constexpr uint8_t operator[](size_t i) const {
        return desc[i];
    }

    template<size_t N2, size_t N3>
    constexpr void MergeArray(const CharArray<N2>& lh, const CharArray<N3>& rh) {
        for (size_t i = 0; i < N2; ++i) {
            desc[i] = lh[i];
        }
        for (size_t i = 0; i < N3; ++i) {
            desc[i + N2] = rh[i];
        }
    }
};

template<class DESC>
static constexpr size_t desc_len = DESC::len;

template<class...DESC>
struct DESC_LEN_SUMMER {
    static constexpr size_t len = (0 + ... + desc_len<DESC>);
};

struct BulkInitPack {
    uint8_t address;
    uint16_t max_pack_size;
    uint8_t interval;
};

struct InterruptInitPack {
    uint8_t address;
    uint16_t max_pack_size;
    uint8_t interval;
};

enum class SynchronousType {
    None,
    Isochronous,
    Adaptive,
    Synchronous
};

enum class IsoEpType {
    Data,
    Feedback,
    ImplicitFeedback
};

struct IsochronousInitPack {
    uint8_t address;
    uint16_t max_pack_size;
    uint8_t interval;
    SynchronousType sync_type;
    IsoEpType endpoint_type;
};

struct ControlInitPack {
    uint8_t address;
    uint16_t max_pack_size;
    uint8_t interval;
};

struct IEndpoint {};
template<class... DESCS>
struct Endpoint : public IEndpoint {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + 7;
    CharArray<len> char_array {
        7,
        5,
    };
    size_t begin = 7;

    constexpr Endpoint(BulkInitPack pack, const DESCS&... decs) {
        char_array[2] = pack.address;
        char_array[3] = 0x02;
        char_array[4] = pack.max_pack_size & 0xff;
        char_array[5] = pack.max_pack_size >> 8;
        char_array[6] = pack.interval;
        (AppendDesc(decs),...);
    }

    constexpr Endpoint(InterruptInitPack pack, const DESCS&... decs) {
        char_array[2] = pack.address;
        char_array[3] = 0x03;
        char_array[4] = pack.max_pack_size & 0xff;
        char_array[5] = pack.max_pack_size >> 8;
        char_array[6] = pack.interval;
        (AppendDesc(decs),...);
    }

    constexpr Endpoint(IsochronousInitPack pack, const DESCS&... decs) {
        char_array[2] = pack.address;
        char_array[3] = 0x01 | (static_cast<uint8_t>(pack.sync_type) << 2) | (static_cast<uint8_t>(pack.endpoint_type) << 4);
        char_array[4] = pack.max_pack_size & 0xff;
        char_array[5] = pack.max_pack_size >> 8;
        char_array[6] = pack.interval;
        (AppendDesc(decs),...);
    }

    constexpr Endpoint(ControlInitPack pack, const DESCS&... decs) {
        char_array[2] = pack.address;
        char_array[3] = 0;
        char_array[4] = pack.max_pack_size & 0xff;
        char_array[5] = pack.max_pack_size >> 8;
        char_array[6] = pack.interval;
        (AppendDesc(decs),...);
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

struct InterfaceInitPack {
    uint8_t interface_no;
    uint8_t alter;
    uint8_t class_;
    uint8_t subclass;
    uint8_t protocol;
    uint8_t str_id;
};
struct InterfaceInitPackClassed {
    uint8_t interface_no;
    uint8_t alter;
    uint8_t protocol;
    uint8_t str_id;
};

struct IInterface {
    static constexpr size_t interface_no_offset = 2;
    static constexpr size_t alter_offset = 3;
    static constexpr size_t num_endpoint_offset = 4;
    static constexpr size_t class_offset = 5;
    static constexpr size_t subclass_offset = 6;
    static constexpr size_t protocol_offset = 7;
    static constexpr size_t str_id_offset = 8;
};
/*
 * when placed into a interface
 * the interface will copy the endpoint's descriptor
 * and increase the num of endpoint
 * so you just need to inherit from @Endpoint class
 *
 * unless you want to modify the interface's behavior,
 * eg: your inherited class or non inhirited class have 2 or more endpoint
 * you have to inherit from @IInterfaceCustom
 * the copy of descriptor will be automatically merged
 * but the num of endpoint will not
*/
struct IInterfaceCustom {};
template<class... DESCS>
struct Interface : public IInterface {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + 9;
    CharArray<len> char_array {
        9,
        4,
    };
    size_t begin = 9;

    constexpr Interface(InterfaceInitPack pack, const DESCS&... descs) {
        char_array[2] = pack.interface_no;
        char_array[3] = pack.alter;
        char_array[4] = 0; // num endpoint
        char_array[5] = pack.class_;
        char_array[6] = pack.subclass;
        char_array[7] = pack.protocol;
        char_array[8] = pack.str_id;
        (AppendDesc(descs),...);
    }

    template<class DESC>
    constexpr void AppendDesc(const DESC& desc) {
        if constexpr (std::is_base_of_v<IEndpoint, DESC>) {
            char_array[4]++;
        }
        else if constexpr (std::is_base_of_v<IInterfaceCustom, DESC>) {
            desc.OnAddToInterface(*this);
        }

        size_t desc_len = desc.len;
        for (size_t i = 0; i < desc_len ; ++i) {
            char_array[begin + i] = desc.char_array[i];
        }
        begin += desc_len;
    }
};

struct InterfaceAssociationInitPack {
    uint8_t function_class;
    uint8_t function_subclass;
    uint8_t function_protocol;
    uint8_t function_str_id;
};
struct IInterfaceAssociation {
    static constexpr size_t first_interface_offset = 2;
    static constexpr size_t interface_count_offset = 3;
};
/*
 * like the @IInterfaceCustom, if your class have 2 or more interface
 * you have to inherit from @IInterfaceAssociationCustom
 * the copy of descriptor will be automatically merged
 * but the num of interface will not
 *
 *  constexpr void OnAddToInterfaceAssociation(auto& association) const
*/
struct IInterfaceAssociationCustom {};
template<class... DESCS>
struct InterfaceAssociation : public IInterfaceAssociation {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + 8;
    CharArray<len> char_array {
        8,
        0xb,
        0, // first interface
        0  // interface count
    };
    size_t begin = 8;

    constexpr InterfaceAssociation(InterfaceAssociationInitPack pack, const DESCS&... desc) {
        char_array[4] = pack.function_class;
        char_array[5] = pack.function_subclass;
        char_array[6] = pack.function_protocol;
        char_array[7] = pack.function_str_id;
        (AppendDesc(desc),...);
    }

    template<class DESC>
    constexpr void AppendDesc(const DESC& desc) {
        if constexpr (std::is_base_of_v<IInterface, DESC>) {
            // TODO: enhance logic
            if (char_array[3] == 0) {
                char_array[2] = desc.char_array[2];
            }
            if (desc.char_array[3] == 0) {
                char_array[3]++;
            }
        }
        else if constexpr (std::is_base_of_v<IInterfaceAssociationCustom, DESC>) {
            desc.OnAddToInterfaceAssociation(*this);
        }

        size_t desc_len = desc.len;
        for (size_t i = 0; i < desc_len ; ++i) {
            char_array[begin + i] = desc.char_array[i];
        }
        begin += desc_len;
    }
};

struct ConfigInitPack {
    uint8_t config_no;
    uint8_t str_id;
    uint8_t attribute;
    uint8_t power;
};
struct IConfig {
    static constexpr size_t len_offset = 0;
    static constexpr size_t type_offset = 1;
    static constexpr size_t total_len_offset = 2;
    static constexpr size_t num_interface_offset = 4;
    static constexpr size_t str_id_offset = 5;
    static constexpr size_t attribute_offset = 6;
    static constexpr size_t power_offset = 7;
};
/*
 * like the @IInterfaceCustom and @IInterfaceAssociationCustom
 * if your class have 2 or more interface or interface association
 * you have to inherit from @IConfigCustom
 * the copy of descriptor will be automatically merged
 *
 * constexpr void OnAddToConfig(auto& config) const
*/
struct IConfigCustom {};
// 1. one @ConfigInitPack
// 2. any @Interface or @InterfaceAssociation
template<class... DESCS>
struct Config : public IConfig {
    static constexpr size_t len = DESC_LEN_SUMMER<DESCS...>::len + 9;
    CharArray<len> char_array {
        9,
        2
    };
    size_t begin = 9;

    constexpr Config(ConfigInitPack pack, const DESCS&... desc) {
        char_array[2] = len & 0xff;
        char_array[3] = len >> 8;
        char_array[4] = 0; // num interface
        char_array[5] = pack.config_no;
        char_array[6] = pack.str_id;
        char_array[7] = pack.attribute;
        char_array[8] = pack.power;
        (AppendDesc(desc),...);

        // check config no
        if (pack.config_no == 0) {
            throw "pack.config_no can not be 0";
        }
    }

    template<class DESC>
    constexpr void AppendDesc(const DESC& desc) {
        if constexpr (std::is_base_of_v<IInterface, DESC>) {
            if (desc.char_array[3] == 0) {
                char_array[4]++;
            }
        }
        else if constexpr (std::is_base_of_v<IInterfaceAssociation, DESC>) {
            char_array[4] += desc.char_array[3];
        }
        else if constexpr (std::is_base_of_v<IConfigCustom, DESC>) {
            desc.OnAddToConfig(*this);
        }

        size_t desc_len = desc.len;
        for (size_t i = 0; i < desc_len ; ++i) {
            char_array[begin + i] = desc.char_array[i];
        }
        begin += desc_len;
    }
};

template<size_t N>
struct CustomDesc {
    static constexpr size_t len = N;
    CharArray<len> char_array;

    constexpr CustomDesc(std::array<int, N> init) {
        if (N != init[0]) {
            throw "len not equal";
        }
        for (size_t i = 0; i < N; ++i) {
            char_array[i] = init[i];
        }
    }
};

