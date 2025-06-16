#pragma once
#include "usb.hpp"
#include <cstddef>
#include <cstdint>

// --------------------------------------------------------------------------------
// CDC
// --------------------------------------------------------------------------------

struct FunctionDesc {
    static constexpr size_t len = 5;
    CharArray<len> char_array {
        len,
        0x24,
        0,
    };

    constexpr FunctionDesc(uint16_t version) {
        char_array[3] = version & 0xff;
        char_array[4] = version >> 8;
    }
};

struct CDCLength {
    static constexpr size_t len = 5;
    CharArray<len> char_array {
        len,
        0x24,
        1,
    };

    constexpr CDCLength(uint8_t capability, uint8_t data_interface) {
        char_array[3] = capability;
        char_array[4] = data_interface;
    }
};

struct CDCManagement {
    static constexpr size_t len = 4;
    CharArray<len> char_array {
        len,
        0x24,
        2,
    };

    constexpr CDCManagement(uint8_t capability) {
        char_array[3] = capability;
    }
};

struct CDCInterfaceSpecify {
    static constexpr size_t len = 5;
    CharArray<len> char_array{
        len,
        0x24,
        6
    };

    constexpr CDCInterfaceSpecify(uint8_t control, uint8_t data) {
        char_array[3] = control;
        char_array[4] = data;
    }
};

struct CDCControlInterface 
: public Interface<FunctionDesc, CDCLength, CDCManagement, CDCInterfaceSpecify, Endpoint<>> {
    constexpr CDCControlInterface(
        InterfaceInitPackClassed pack,
        FunctionDesc function_desc,
        CDCLength cdc_length,
        CDCManagement cdc_management,
        CDCInterfaceSpecify cdc_interface_specify,
        Endpoint<> update_endpoint
    ) : Interface<FunctionDesc, CDCLength, CDCManagement, CDCInterfaceSpecify, Endpoint<>>(
        InterfaceInitPack{
            pack.interface_no,
            pack.alter,
            0x02,
            0x02,
            pack.protocol,
            pack.str_id
        },
        function_desc,
        cdc_length,
        cdc_management,
        cdc_interface_specify,
        update_endpoint
    ) {}
};

// need endpoints, see @Endpoint<>
template<class... DESCS>
struct CDCDataInterface : public Interface<DESCS...> {
    constexpr CDCDataInterface(
        InterfaceInitPackClassed pack,
        const DESCS&... descs
    ) : Interface<DESCS...>(InterfaceInitPack{
        pack.interface_no,
        pack.alter,
        0x0a,
        0x00,
        pack.protocol,
        pack.str_id
    }, descs...) {}
};
