#include "usbpp/usb.hpp"
#include "usbpp/uac2.hpp"
#include "usbpp/hid.hpp"
#include "usbpp/cdc.hpp"

static constexpr auto config=
Config{
    ConfigInitPack{
        1, 0, 0x80, 250
    },
    UAC2_InterfaceAssociation{
        UAC2_InterfaceAssociation_InitPack{
            .str_id = 0,
            .protocol = 0x20
        },
        AudioControlInterface{
            InterfaceInitPackClassed{
                .interface_no = 0,
                .alter = 0,
                .protocol = 0x20,
                .str_id = 0
            },
            AudioFunction{
                AudioFunctionInitPack{
                    0x0200, 1, 0
                },
                Clock{
                    3, 2, 3, 0, 0
                },
                InputTerminal{
                    1, 0x0101, 0, 3, 0, 0, {2, 0x3, 0}
                },
                FeatureUnit<3>{
                    FeatureUnitInitPack{
                        4, 1, 0
                    },
                    {0xf, 0xf, 0xf}
                },
                OutputTerminal{
                    2, 0x0301, 0, 4, 3, 0, 0
                }
            }
        },
        AudioStreamInterface{
            InterfaceInitPackClassed{
                .interface_no = 1,
                .alter = 0,
                .protocol = 0x20,
                .str_id = 0
            },
            TerminalLink{
                1, 0, 1, 1, ChannelInitPack{
                    2, 3, 0
                }
            },
            AudioStreamFormat{
                1, 4, 32
            },
            Endpoint{
                IsochronousInitPack{
                    1, 1024, 1, SynchronousType::Isochronous, IsoEpType::Data
                },
                CustomDesc{
                    std::array{8, 0x25, 0x01, 0, 0, 0, 0, 0}
                }
            },
            Endpoint{
                IsochronousInitPack{
                    0x81, 4, 1, SynchronousType::None, IsoEpType::Feedback
                }
            }
        }
    },
    InterfaceAssociation{
        InterfaceAssociationInitPack{
            2, 2, 1, 0
        },
        CDCControlInterface{
            InterfaceInitPackClassed{
                2, 0, 1, 0
            },
            FunctionDesc{
                0x0110
            },
            CDCLength{
                0, 3
            },
            CDCManagement{
                2
            },
            CDCInterfaceSpecify{
                2, 3
            },
            Endpoint{
                InterruptInitPack{
                    0x83, 64, 4
                }
            }
        },
        CDCDataInterface{
            InterfaceInitPackClassed{
                3, 0, 0, 0
            },
            Endpoint{
                BulkInitPack{
                    0x02, 64, 4
                }
            },
            Endpoint{
                BulkInitPack{
                    0x82, 64, 4
                }
            }
        }
    },
    InterfaceAssociation{
        InterfaceAssociationInitPack{
            .function_class = 3,
            .function_subclass = 0,
            .function_protocol = 0,
            .function_str_id = 0
        },
        HID_Interface{
            InterfaceInitPackClassed{
                .interface_no = 4,
                .alter = 0,
                .protocol = 0,
                .str_id = 0
            },
            HID_Descriptor<1>{
                HID_Descriptor_InitPack{
                    .bcd_hid = 0x0111,
                    .country_code = 0,
                },
                {
                    HID_DescriptorLengthDesc{
                        .type = 0x22,
                        .length = 32
                    }
                }
            },
            Endpoint{
                InterruptInitPack{
                    .address = 4,
                    .max_pack_size = 3,
                    .interval = 4
                }
            },
            Endpoint{
                InterruptInitPack{
                    .address = 0x84,
                    .max_pack_size = 3,
                    .interval = 4
                }
            }
        }
    }
};

extern "C" {
const uint8_t* MyCfgDescr_HS = config.char_array.desc;
const uint32_t config_desc_len = config.char_array.desc_len;
}
