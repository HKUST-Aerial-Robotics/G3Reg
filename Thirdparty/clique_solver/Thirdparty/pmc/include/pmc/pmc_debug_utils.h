//
// Created by jnshi on 10/2/22.
//

#ifndef PMC_DEBUG_UTILS_H_
#define PMC_DEBUG_UTILS_H_

#ifdef PMC_ENABLE_DEBUG
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) do {} while (0)
#endif

#endif //PMC_DEBUG_UTILS_H_
