cmake_minimum_required(VERSION 3.11)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(dfu)

# download JUCE
include(FetchContent)
FetchContent_Declare(
    JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    SHA256 60fa1d4554fc58e166e8e26c655b0a5cb3a5798b6e6daaea2f957a741d5ba422
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/juce
)
add_subdirectory(juce)

juce_add_gui_app(DFU
    VERSION 0.0.1                       # Set this if the app version is different to the project version
    # ICON_BIG ...                      # ICON_* arguments specify a path to an image file to use as an icon
    # ICON_SMALL ...
    # DOCUMENT_EXTENSIONS ...           # Specify file extensions that should be associated with this app
    # COMPANY_NAME ...                  # Specify the name of the app's author
    PRODUCT_NAME "Gui App Example")     # The name of the final executable, which can differ from the target name

file(GLOB_RECURSE SRCS "src/*.cpp")
target_sources(DFU PRIVATE ${SRCS})

target_compile_definitions(DFU
    PRIVATE
        # JUCE_WEB_BROWSER and JUCE_USE_CURL would be on by default, but you might not need them.
        JUCE_WEB_BROWSER=0  # If you remove this, add `NEEDS_WEB_BROWSER TRUE` to the `juce_add_gui_app` call
        JUCE_USE_CURL=0     # If you remove this, add `NEEDS_CURL TRUE` to the `juce_add_gui_app` call
        JUCE_APPLICATION_NAME_STRING="$<TARGET_PROPERTY:DFU,JUCE_PRODUCT_NAME>"
        JUCE_APPLICATION_VERSION_STRING="$<TARGET_PROPERTY:DFU,JUCE_VERSION>")

# juce_add_binary_data(GuiAppData SOURCES ...)

target_link_directories(DFU
    PUBLIC
        "lib/"
)
target_include_directories(DFU
    PUBLIC
        "include/"
        "juce/modules/"
)

target_link_libraries(DFU
    PRIVATE
        # GuiAppData            # If we'd created a binary data target, we'd link to it here
        juce::juce_gui_extra
        libusb-1.0
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)
