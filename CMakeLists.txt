cmake_minimum_required(VERSION 3.22)
set(SRCS
    "src/lmic/lmic.c"
    "src/lmic/oslmic.c"
    "src/lmic/radio.c"
    "src/hal/hal.c"
    "src/aes/ideetron/AES-128_V10.cpp"
    "src/aes/lmic.c"
    "src/aes/other.c"
)

idf_component_register(SRCS "${SRCS}"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)