cmake_minimum_required(VERSION 3.16)
project(GSHeadless LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ★ ビルド生成物はプロジェクト直下に出す
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_definitions(-DSH_HEADLESS=1) # 描画コードを無効化

# 依存ヘッダ（先頭に当てる）
include_directories(
  ${CMAKE_SOURCE_DIR}/vecmath
)

# あなたのソース（必要なら追加ください）
set(GS_SOURCES
  GSModel.h GSModel_core.cpp GSModel_generate.cpp GSModel_train.cpp GSModel_dump.cpp
  GSModelTest.h GSModelTest.cpp GSModelTestMain.cpp
  HumanBody.h HumanBody.cpp
  SimpleHuman.h SimpleHuman.cpp
  BVH.h BVH.cpp
)

add_library(gsmodel ${GS_SOURCES})
target_compile_definitions(gsmodel PRIVATE SH_HEADLESS=1)

add_executable(app_headless main_headless.cpp)
target_link_libraries(app_headless PRIVATE gsmodel)

# デフォルトはRelease
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
