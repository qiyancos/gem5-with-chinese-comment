/*
 * Copyright (c) 2020 Peking University
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __MEM_CACHE_PREFETCH_FILTER_DEBUG_FLAG_HH__
#define __MEM_CACHE_PREFETCH_FILTER_DEBUG_FLAG_HH__

#include <cstdint>
#include <string>

#include <stdio.h>

#include "base/types.hh"

namespace prefetch_filter {

extern Tick debugStartTick_;

extern Tick tickNow_;

} // namespace prefetch_filter

// 可以在这里控制总开关，也可以在各自文件中添加
#define USE_CHECK 1
#define DEBUG_FILTER 1
#define DEBUG_CACHE 1
#define DEBUG_TEMP 1

// 下面是带信息的检查宏函数
#ifdef USE_CHECK

#define CHECK_RET_EXIT(expr, info, ...) { \
    int errCode = expr; \
    panic_if(errCode < 0, (std::string(__FILE__) + "[%d]: " + \
            info + " In [%s]." + " Error code = %d\n").c_str(), \
            __LINE__, ##__VA_ARGS__, __func__, errCode); \
}

#define CHECK_ARGS_EXIT(expr, info, ...) { \
    panic_if(!(expr), (std::string(__FILE__) + "[%d]: " + \
            info + " In [%s].\n").c_str(), __LINE__, \
            ##__VA_ARGS__, __func__); \
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

#else

#define CHECK_RET_EXIT(expr, info, ...) { expr; }

#define CHECK_ARGS_EXIT(expr, info, ...) {}

#define CHECK_RET(expr, info, ...) { expr; }

#define CHECK_WARN(expr, info, ...) {}

#define CHECK_ARGS(expr, info, ...) {}

#endif

// 下面是Prefetch Filter中的debug信息宏函数
#ifdef DEBUG_FILTER

#define DEBUG_PF(indentation, info, ...) { \
    if (prefetch_filter::tickNow_ >= prefetch_filter::debugStartTick_) { \
        fprintf(stderr, "[Debug] "); \
        for (int i = 0; i < indentation; i++) { \
            fprintf(stderr, "    "); \
        } \
        fprintf(stderr, (std::string(">> ") + info + \
                " In [%s:%d].\n").c_str(), ##__VA_ARGS__, \
                __func__, __LINE__); \
        fflush(stderr); \
    } \
}


#define DEBUG_PF_PLINE() { \
    if (prefetch_filter::tickNow_ >= prefetch_filter::debugStartTick_) { \
        fprintf(stderr, "\n"); \
    } \
}

#else

#define DEBUG_PF(indentation, info, ...) {}
#define DEBUG_PF_PLINE() {}

#endif

// 下面是其他结构相关处理信息的Debug函数
#ifdef DEBUG_CACHE

#define DEBUG_MEM(info, ...) { \
    if (prefetch_filter::tickNow_ >= prefetch_filter::debugStartTick_) { \
        fprintf(stderr, (std::string("[Debug] -- ") + info + \
                "\n").c_str(), ##__VA_ARGS__); \
    } \
}

#else

#define DEBUG_MEM(info, ...) {}

#endif

// 下面是针对特殊错误使用的Debug函数
#ifdef DEBUG_TEMP

#define DEBUG_LINE() { \
    if (prefetch_filter::tickNow_ >= prefetch_filter::debugStartTick_) { \
        fprintf(stderr, (std::string("[Debug] == Breakpoint in") + \
                " %s:%d.\n").c_str(), __func__, __LINE__); \
    } \
}

#define DEBUG_EXIT(expr) { \
    if (expr) { \
        fprintf(stderr, (std::string("[Debug] == Exitpoint in") + \
                "%s:%d.\n").c_str(), __func__, __LINE__); \
        exit(0); \
    } \
}

#else

#define DEBUG_LINE() {}

#define DEBUG_EXIT() {}

#endif

#endif // __MEM_CACHE_PREFETCH_FILTER_DEBUG_FLAG_HH__
