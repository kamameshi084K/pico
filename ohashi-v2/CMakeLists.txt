# 必要なCMakeのバージョンを指定
cmake_minimum_required(VERSION 3.12)

# Pico SDKをインポート
include(pico_sdk_import.cmake)

# オプションのエクストラライブラリをインポート
include(pico_extras_import_optional.cmake)


# プロジェクト名と必要なC標準を設定
project(pico_extras C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pico SDKのパスとボードを設定
set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
set(PICO_BOARD "pico")

# pico-dualsenseの場所を指定してサブディレクトリとして追加
add_subdirectory("C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk/lib/pico-dualsense-main" pico-dualsense)

# pico-dualsense用のbtstackのインクルードディレクトリを指定
include_directories("C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk/lib/pico-dualsense-main/include")

# SDKのバージョンチェック
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# プロジェクトのソースディレクトリを設定
set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# SDKを初期化
pico_sdk_init()

# コンパイルオプションを追加
add_compile_options(
    -Wall                  # 全ての警告を表示
    -Wno-format            # intとint32_tのフォーマット警告を無視
    -Wno-unused-function   # 未使用関数の警告を無視
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized) # 未初期化の警告を無視（GCCの場合）
endif()

# 必要に応じて追加のサブディレクトリを追加
add_subdirectory(ohashi-v2)




