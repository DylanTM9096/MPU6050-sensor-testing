# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/dylan/esp/esp-idf/components/bootloader/subproject"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/tmp"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/src/bootloader-stamp"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/src"
  "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/dylan/esp/esp_proj/self-balancing-robot/MPU6050-sensor-testing/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
