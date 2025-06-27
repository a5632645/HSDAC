#include "MainComponent.h"
#include "juce_core/juce_core.h"
#include "juce_core/system/juce_PlatformDefs.h"
#include "juce_events/juce_events.h"
#include "juce_gui_basics/juce_gui_basics.h"
#include "libusb.h"
#include <chrono>
#include <exception>
#include <memory>
#include <thread>

static constexpr int kVendorID = 0x1A86;
static constexpr int kUACProductID = 0x1;
static constexpr int kBootProductID = 0x2;

static constexpr struct _UNNAME_STRUCT_16 {
    static constexpr int kProgramReady = 1;
    static constexpr int kVerifyReady = 2;
    static constexpr int kPrograming = 3;
    static constexpr int kVerifying = 4;
    static constexpr int kFinished = 5;
} BootloadState;

struct TimerGuide {
    juce::Timer& timer_;

    explicit TimerGuide(juce::Timer& t)
    : timer_(t) {
        t.stopTimer();
    }

    ~TimerGuide() {
        timer_.startTimerHz(1);
    }
};

//==============================================================================
MainComponent::MainComponent() {
    if (libusb_init(&usb_context_) < 0) {
        ShowError("failed to init libusb", true);
    }

    device_state_.setText("unconnect", juce::dontSendNotification);
    addAndMakeVisible(device_state_);

    update_button_.setButtonText(juce::String::fromUTF8("上传固件"));
    addAndMakeVisible(update_button_);
    update_button_.onClick = [this] {
        TimerGuide _{ *this };
        if (current_device_ == eMyDevice::UAC) {
            ResetToBootloader();
            StartBootloaderFlash();
        }
        else if (current_device_ == eMyDevice::BOOT) {
            StartBootloaderFlash();
        }
        else {
            ShowError("you should connect to device first", false);
        }
    };

    firmware_path_.setText("FW: NULL", juce::dontSendNotification);
    addAndMakeVisible(firmware_path_);

    firmware_path_choose_.setButtonText("...");
    firmware_path_choose_.onClick = [this] {
        firmware_chooser_ = std::make_unique<juce::FileChooser>(
            "Firmware", juce::File{}, "*.hex;*.bin"
        );
        firmware_chooser_->launchAsync(0, [this](const juce::FileChooser& fc) {
            this->firmware_path_.setText(fc.getResult().getFullPathName(), juce::dontSendNotification);
        });
    };
    addAndMakeVisible(firmware_path_choose_);

    test_button_.setButtonText("test");
    addAndMakeVisible(test_button_);

    setSize (600, 400);

    // begin auto connect
    startTimerHz(1);
}

MainComponent::~MainComponent() {
    if (usb_device_handle_ != nullptr) {
        libusb_close(usb_device_handle_);
    }
    libusb_exit(usb_context_);
}

//==============================================================================
void MainComponent::paint (juce::Graphics& g) {
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));
}

void MainComponent::resized() {
    auto b = getLocalBounds();
    {
        auto top = b.removeFromTop(30);
        device_state_.setBounds(top.removeFromLeft(100));
        update_button_.setBounds(top.removeFromLeft(100));
        firmware_path_choose_.setBounds(top.removeFromRight(100));
        firmware_path_.setBounds(top);
    }
    {
        auto top = b.removeFromTop(30);
        test_button_.setBounds(top.removeFromLeft(100));
    }
}

void MainComponent::ShowError(const juce::String& error, bool exit) {
    juce::NativeMessageBox::showAsync(
    juce::MessageBoxOptions{}
        .withMessage(error)
        .withParentComponent(this)
        .withIconType(juce::MessageBoxIconType::WarningIcon)
        .withTitle("error")
        .withButton("ok"),
    [exit](int) {
        if (exit) {
            std::terminate(); 
        }
    });
}

void MainComponent::timerCallback() {
    if (usb_device_handle_ == nullptr) {
        usb_device_handle_ = libusb_open_device_with_vid_pid(usb_context_, kVendorID, kUACProductID);
        if (usb_device_handle_ != nullptr) {
            DBG("[AC] connect to uac");
            current_device_ = eMyDevice::UAC;
            OnUsbDeviceConnect(eMyDevice::UAC);
            return;
        }
        
        usb_device_handle_ = libusb_open_device_with_vid_pid(usb_context_, kVendorID, kBootProductID);
        if (usb_device_handle_ != nullptr) {
            DBG("[AC] connect to bootloader");
            current_device_ = eMyDevice::BOOT;
            OnUsbDeviceConnect(eMyDevice::BOOT);
            return;
        }
    }
    else {
        if (!IsDeviceActive(current_device_)) {
            libusb_close(usb_device_handle_);
            DBG("[AC] device " << (current_device_ == eMyDevice::UAC ? "uac " : "bootloader") << " disconnect");
            OnUsbDeviceDisconnect(current_device_);
            current_device_ = eMyDevice::None;
            usb_device_handle_ = nullptr;    
        }
    }
}

void MainComponent::StartBootloaderFlash() {
    if (current_device_ != eMyDevice::BOOT) {
        DBG("[BOOT] invalid device");
        ShowError("invalid device", false);
        return;
    }

    // read firmware file
    auto file = firmware_path_.getText();
    juce::File bin{file};
    juce::FileInputStream fis{bin};

    // select hid interface
    int err = libusb_claim_interface(usb_device_handle_, 0);
    if (err != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to claim interface 0: " << libusb_error_name(err);
        DBG(err_str);
        ShowError(err_str, false);
        return;
    }

    // transmite, no hid report id here
    // every package have a 0x0 means it's a data pack
    for (int epo = 0; epo < 3; ++epo) {
        if (epo == 0) {
            // download
            StateReq state = GetBootState();
            if (state.error_ != 0) return;
            if (state.val != BootloadState.kProgramReady) return;

            StateReq last = GetLastAppState();
            if (last.error_ != 0 && last.val == 1) {
                DBG("[BOOT] last app flash failed");
            }
        }
        else if (epo == 1) {
            // download end
            // verify
            StateReq state = GetBootState();
            if (state.error_ != 0) return;
            if (state.val != BootloadState.kVerifyReady) return;
            fis.setPosition(0);
        }
        else {
            // verify end
            StateReq state = GetBootState();
            if (state.error_ != 0) return;
            if (state.val != BootloadState.kFinished) return;

            StateReq last = GetLastAppState();
            if (last.error_ != 0) return;
            if (last.val == 0) {
                DBG("[BOOT] program success");
            }
            else {
                DBG("[BOOT] program failed!");
            }
            SendEndPackToBootloader();
            
            break;
        }

        int num_tx{};
        unsigned char temp[260]{};
        int numpackage = (int)std::ceil(fis.getTotalLength() / 256.0);
        int curr_pack = 1;
        while (!fis.isExhausted()) {
            int bytes_read = fis.read(temp + 4, 256);
            std::fill(temp + 4 + bytes_read, temp + 260, 0);

            err = libusb_interrupt_transfer(usb_device_handle_, 1, temp, 260, &num_tx, 1000);
            if (err != 0) {
                juce::String err_str;
                err_str << "[BOOT]failed to send package " << curr_pack << '/' << numpackage << " because " << libusb_error_name(err);
                DBG(err_str);
                ShowError(err_str, false);
                return;
            }

            ++curr_pack;
        }

        SendEndPackToBootloader();
    }
    

    err = libusb_release_interface(usb_device_handle_, 0);
    if (err != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to release interface 0: " << libusb_error_name(err);
        DBG(err_str);
        ShowError(err_str, false);
        return;
    }
}

void MainComponent::OnUsbDeviceConnect(eMyDevice device) {
    if (device == eMyDevice::BOOT) {
        device_state_.setText("BOOT", juce::dontSendNotification);
    }
    else if (device == eMyDevice::UAC) {
        device_state_.setText("UAC", juce::dontSendNotification);
    }
}

void MainComponent::OnUsbDeviceDisconnect(eMyDevice device) {
    if (device == eMyDevice::BOOT) {

    }
    else if (device == eMyDevice::UAC) {

    }
    device_state_.setText("Disconnect", juce::dontSendNotification);
}

void MainComponent::ResetToBootloader() {
    libusb_claim_interface(usb_device_handle_, 4);
    unsigned char data[3] {
        1, 0x12, 0x34
    };
    int num_tx{};
    libusb_interrupt_transfer(usb_device_handle_, 4, data, 3, &num_tx, 1000);

    // reseted
    libusb_close(usb_device_handle_);
    usb_device_handle_ = nullptr;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // reconnect
    current_device_ = eMyDevice::None;
    timerCallback();
}

void MainComponent::SendEndPackToBootloader() {
    unsigned char temp[260] {
        0xff,
        0xff,
        0xff,
        0xff
    };
    int num_tx{};
    int err = libusb_interrupt_transfer(usb_device_handle_, 1, temp, 260, &num_tx, 1000);
    if (err != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to send end package: " << libusb_error_name(err);
        DBG(err_str);
    }
}

MainComponent::StateReq MainComponent::GetBootState() {
    StateReq req;

    unsigned char temp[260] {
        0x00,
        0x00,
        0x00,
        0xff,
    };
    int num_tx{};
    req.error_ = libusb_interrupt_transfer(usb_device_handle_, 1, temp, 260, &num_tx, 1000);
    if (req.error_ != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to send get state package: " << libusb_error_name(req.error_);
        DBG(err_str);
        return req;
    }

    req.error_ = libusb_interrupt_transfer(usb_device_handle_, 0x81, temp, 1, &num_tx, 1000);
    if (req.error_ != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to read get state package: " << libusb_error_name(req.error_);
        DBG(err_str);
        return req;
    }

    req.val = temp[0];
    return req;
}

MainComponent::StateReq MainComponent::GetLastAppState() {
    StateReq req;

    unsigned char temp[260] {
        0x00,
        0x00,
        0xff,
        0x00,
    };
    int num_tx{};
    req.error_ = libusb_interrupt_transfer(usb_device_handle_, 1, temp, 260, &num_tx, 1000);
    if (req.error_ != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to send get last app state package: " << libusb_error_name(req.error_);
        DBG(err_str);
        return req;
    }

    req.error_ = libusb_interrupt_transfer(usb_device_handle_, 0x81, temp, 1, &num_tx, 1000);
    if (req.error_ != 0) {
        juce::String err_str;
        err_str << "[BOOT]failed to read get last app state package: " << libusb_error_name(req.error_);
        DBG(err_str);
        return req;
    }

    req.val = temp[0];
    return req;
}

bool MainComponent::IsDeviceActive(eMyDevice device) {
    libusb_device **list;
    ssize_t cnt;
    cnt = libusb_get_device_list(usb_context_, &list);
    if (cnt < 0) {
        return false;
    }

    int target_vid = kVendorID;
    int target_pid = device == eMyDevice::UAC ? kUACProductID : kBootProductID;

    bool found = false;
    for (ssize_t i = 0; i < cnt; i++) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(list[i], &desc);
        if (r == 0 && desc.idVendor == target_vid && desc.idProduct == target_pid) {
            found = true;
            break;
        }
    }

    libusb_free_device_list(list, 1);

    return found; // 1 表示已连接，0 表示未连接
}