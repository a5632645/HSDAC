#include "uac_widget.hpp"
#include "juce_events/juce_events.h"
#include "libusb.h"

UACWidget::UACWidget() {
    latency_label_.setText("latency", juce::dontSendNotification);
    addAndMakeVisible(latency_label_);
    latency_.setRange(256, 1024, 1);
    latency_.setValue(1024);
    addAndMakeVisible(latency_);
    latency_.onValueChange = [this] {
        int latency = static_cast<int>(latency_.getValue());
        pending_request_.insert(Request{
            2,
            latency >> 8,
            latency & 0xff
        });
    };
}

void UACWidget::resized() {
    auto b = getLocalBounds();
    {
        auto t = b.removeFromTop(20);
        latency_label_.setBounds(t.removeFromLeft(100));
        latency_.setBounds(t);
    }
}

void UACWidget::OnUACDeviceConnect(libusb_device_handle* handle) {
    uac_device_handle_ = handle;
    startTimerHz(100);
}

void UACWidget::OnUACDevoceDisconnect() {
    uac_device_handle_ = nullptr;
    stopTimer();
}

void UACWidget::timerCallback() {
    if (uac_device_handle_ != nullptr) {
        libusb_claim_interface(uac_device_handle_, 4);
        for (auto& req : pending_request_) {
            unsigned char buffer[] {
                static_cast<unsigned char>(req.type),
                static_cast<unsigned char>(req.reg),
                static_cast<unsigned char>(req.val),
            };
            int num_tx{};
            libusb_interrupt_transfer(uac_device_handle_, 4, buffer, 3, &num_tx, 1000);
        }
        pending_request_.clear();
        libusb_release_interface(uac_device_handle_, 4);
    }
}