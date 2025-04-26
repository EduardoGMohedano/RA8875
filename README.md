RA8875 Driver

The intention of this driver is to be used along with LVGL library, so it only supports drawing pixels function using 16 color depth

TARGET
This driver has been used and tested for ESP32 and ESP32-S3 platforms. 

BUILDING

PlatformIO
You can directly clone the repo on a new created PlatformIO project and compile directtly. Just make sure that you add lvgl 9.2.2 version as a dependency

ESP-IDF VS Code extension
You can clone the repo on a new created project, just make sure to include all the files in your CMake file.

Example:
You can directly flash the example in main.c file and you will get a demo running. The demo shows 9 arcs loading across the whole screen
