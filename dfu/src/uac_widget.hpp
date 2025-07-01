/*
 * TODO: fix crash after bootloader upload
 *       fix message lost (uac iso take all the usb frames)
*/

#pragma once
#include "juce_events/juce_events.h"
#include <cstddef>
#include <functional>
#include <juce_gui_basics/juce_gui_basics.h>
#include <libusb.h>
#include <unordered_set>

class UACWidget : public juce::Component, private juce::Timer {
public:
    struct Request {
        int type;
        int reg;
        int val;

        bool operator==(const Request& other) const {
            return type == other.type
                && reg == other.reg
                && val == other.val;
        }
    };

    struct Requesthash {
        size_t operator()(const Request& r) const {
            return std::hash<int>{}(r.reg + r.val + r.type);
        }
    };

    UACWidget();
    void resized() override;

    void OnUACDeviceConnect(libusb_device_handle* handle);
    void OnUACDevoceDisconnect();

private:
    void timerCallback() override;

    libusb_device_handle* uac_device_handle_{};
    std::unordered_set<Request, Requesthash> pending_request_;
    std::unordered_set<Request, Requesthash> temp_;

    juce::Label latency_label_;
    juce::Slider latency_;
};