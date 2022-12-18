# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/study/Espressif/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix/tmp"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix/src/bootloader-stamp"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix/src"
  "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/study/Espressif/Espressif/esp/Project_GPS/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
