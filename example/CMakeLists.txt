# Минимально потребная версия Cmake
cmake_minimum_required(VERSION 3.0)


# ----  Общие настройки -----
# Название проекта и указание основного языка
project(example LANGUAGES C)
# Этот инклуд реализует парсинг файлов и всасывание результатов в переменную
include(../dap-sdk/cmake/ReadMKFile.cmake)

# Общие настройки
set(CMAKE_ENABLE_EXPORTS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_C_STANDARD 11)

SET( CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR})
SET( CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
SET( CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
add_definitions("-DDAP_VERSION=\"${VERSION_MAJOR}-${VERSION_MINOR}.${VERSION_PATCH}\"")

# Инклуд с определением операционки и выставлением различных дефайнов общего назначения
# А так же выставление флагов для релиза и для отладки
include(../dap-sdk/cmake/OS_Detection.cmake)
# Перечисление модулей DAP SDK которые нам нужны
set(DAPSDK_MODULES "core crypto io app-cli")
# Подключение DAP SDK
add_subdirectory(dap-sdk)

# Подключаем библиотеку DRS
add_subdirectory(libdrs)

#-- Определяем исполняемый файл и библиотеки к нему
add_executable(${PROJECT_NAME} main.c)
target_link_libraries(${PROJECT_NAME} dap_core drs)

# Определяем, куда будем ставить пример. Нужно для удаленной отладки в основном
INSTALL(TARGETS example DESTINATION /root)
