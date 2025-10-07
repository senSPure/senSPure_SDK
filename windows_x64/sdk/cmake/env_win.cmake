set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "Configs" FORCE)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

# generate version.rc
set(LEGAL_COPYRIGHT "Copyright (C) 2023 TOPPAN Holdings Inc.")
configure_file("${ENV_DIR}/vs_version.rc.in" "${CMAKE_CURRENT_SOURCE_DIR}/vs_version.rc")
set(VSRC ${VSRC} vs_version.rc)
