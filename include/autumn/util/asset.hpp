#pragma once

#include <cstddef>
#include <cstdint>

#ifndef _AUTUMN_ASSET_H_
#define _AUTUMN_ASSET_H_

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
    std::uint8_t* buf;
    std::size_t size;
} asset;
}

#define ASSET(x)                                                                                   \
    extern "C" {                                                                                   \
    extern std::uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                 \
    }                                                                                              \
    asset x = {_binary_static_##x##_start, (std::size_t)_binary_static_##x##_size}

#define ASSET_LIB(x)                                                                               \
    extern "C" {                                                                                   \
    extern std::uint8_t _binary_static_lib_##x##_start[], _binary_static_lib_##x##_size[];         \
    }                                                                                              \
    asset x = {_binary_static_lib_##x##_start, (std::size_t)_binary_static_lib_##x##_size}

#endif // _AUTUMN_ASSET_H_
