#ifndef DEFINES_H
#define DEFINES_H

#define LOG_LEVEL_C 4

#if LOG_LEVEL_C >= 1
#define LOG_ERROR_C(x) std::cerr << x << std::endl
#else
#define LOG_ERROR_C(x)
#endif

#if LOG_LEVEL_C >= 2
#define LOG_WARN_C(x) std::cerr << x << std::endl
#else
#define LOG_WARN_C(x)
#endif

#if LOG_LEVEL_C >= 3
#define LOG_INFO_C(x) std::cout << x << std::endl
#else
#define LOG_INFO_C(x)
#endif

#if LOG_LEVEL_C >= 4
#define LOG_DEBUG_C(x) std::cout << x << std::endl
#else
#define LOG_DEBUG_C(x)
#endif



#endif //DEFINES_H