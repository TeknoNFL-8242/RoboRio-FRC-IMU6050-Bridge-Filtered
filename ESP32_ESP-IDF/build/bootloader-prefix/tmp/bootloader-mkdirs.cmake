# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/cppsever/esp-idf-v5.2.6/components/bootloader/subproject"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/tmp"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/src/bootloader-stamp"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/src"
  "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/cppsever/Desktop/ESP32_ESP-IDF/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
