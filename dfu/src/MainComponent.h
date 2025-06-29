/*
 * TODO: fix memory leak
*/

#pragma once

// CMake builds don't use an AppConfig.h, so it's safe to include juce module headers
// directly. If you need to remain compatible with Projucer-generated builds, and
// have called `juce_generate_juce_header(<thisTarget>)` in your CMakeLists.txt,
// you could `#include <JuceHeader.h>` here instead, to make all your module headers visible.
#include "juce_core/juce_core.h"
#include "juce_events/juce_events.h"
#include "juce_gui_basics/juce_gui_basics.h"
#include <juce_gui_extra/juce_gui_extra.h>
#include <libusb.h>
#include <memory>
#include "uac_widget.hpp"

enum class eMyDevice {
    None,
    UAC,
    BOOT
};

class MainComponent final : public juce::Component, private juce::Timer {
public:
    static constexpr int kInitAutoConnectDelayMs = 100;
    static constexpr int kMaxAutoConnectDelayMs = 5000;

    MainComponent();
    ~MainComponent() override;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void timerCallback() override;
    void ShowError(const juce::String& error, bool exit);
    void StartBootloaderFlash();
    void ResetToBootloader();
    void OnUsbDeviceConnect(eMyDevice device);
    void OnUsbDeviceDisconnect(eMyDevice device);
    bool IsDeviceActive(eMyDevice device);

    struct StateReq {
        int error_;
        int val;
    };
    StateReq GetBootState();
    StateReq GetLastAppState();
    void SendEndPackToBootloader();

    libusb_context* usb_context_{};
    libusb_device_handle* usb_device_handle_{};
    eMyDevice current_device_{ eMyDevice::None };

    juce::Label device_state_;
    juce::TextButton update_button_;
    juce::Label firmware_path_;
    juce::TextButton firmware_path_choose_;
    std::unique_ptr<juce::FileChooser> firmware_chooser_;

    UACWidget uac_;
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
