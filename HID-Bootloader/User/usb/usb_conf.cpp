#include "../tpusb/usb.hpp"
#include "../tpusb/hid.hpp"

static constexpr auto usb_config =
Config{
    ConfigInitPack{
        .config_no = 1,
        .str_id = 0,
        .attribute = 0x80,
        .power = 250
    },
    HID_Interface{
        InterfaceInitPackClassed{
            .interface_no = 0,
            .alter = 0,
            .protocol = 0,
            .str_id = 0
        },
        HID_Descriptor<1>{
            HID_Descriptor_InitPack{
                .bcd_hid = 0x0110,
                .country_code = 0
            },
            {
                HID_DescriptorLengthDesc{
                    .type = 0x22,
                    .length = 32,
                }
            }
        },
        Endpoint{
            InterruptInitPack{
                .address = 1,
                .max_pack_size = 512,
                .interval = 1
            }
        },
        Endpoint{
            InterruptInitPack{
                .address = 0x81,
                .max_pack_size = 512,
                .interval = 1
            }
        }
    }
};

extern "C" {

const uint8_t* MyCfgDescr_HS = usb_config.char_array.desc;

}