/*
 * Copyright (c) 2020 Peking University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Rock Lee
 */

#ifndef __MEM_CACHE_PREFETCH_FILTER_PROGRAM_HELPER_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PROGRAM_HELPER_HH__

#include <cstdint>
#include <string>

#include <stdio.h>

namespace prefetch_filter {

#define CHECK_RET_EXIT(expr, info, ...) { \
    int errCode = expr; \
    if (errCode < 0) { \
        fprintf(stderr, (std::string("[ERROR] ") + __FILE__ + "[%d]: " + \
                info + " In [%s]." + " Error code = %d\n").c_str(), \
                __LINE__, ##__VA_ARGS__, __func__, errCode); \
        exit(errCode); \
    } \
}

#define CHECK_ARGS_EXIT(expr, info, ...) { \
    if (!(expr)) { \
        fprintf(stderr, (std::string("[ERROR] ") + __FILE__ + "[%d]: " + \
                info + " In [%s].\n").c_str(), __LINE__, \
                ##__VA_ARGS__, __func__); \
        exit(-1); \
    } \
}

#define CHECK_RET(expr, info, ...) { \
    int retCode = expr; \
    if (retCode < 0) { \
        fprintf(stderr, (std::string("[ERROR] ") + __FILE__ + "[%d]: " + \
                info + " In [%s]." + " Return %d\n").c_str(), \
                __LINE__, ##__VA_ARGS__, __func__, retCode); \
        return retCode; \
    } \
}

#define CHECK_WARN(expr, info, ...) { \
    if (!(expr)) { \
        fprintf(stderr, (std::string("[WARN] ") + __FILE__ + "[%d]: " + \
            info + " In [%s].\n").c_str(), __LINE__, \
            ##__VA_ARGS__, __func__); \
    } \
}

#define CHECK_ARGS(expr, info, ...) { \
    if (!(expr)) { \
        fprintf(stderr, (std::string("[ERROR] ") + __FILE__ + "[%d]: " + \
                info + " In [%s].\n").c_str(), __LINE__, \
                ##__VA_ARGS__, __func__); \
        return -1; \
    } \
}

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PROGRAM_HELPER_HH__
