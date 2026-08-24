#ifndef EXO_LOGGER_H_
#define EXO_LOGGER_H_

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

inline void exo_log_printf(const char *format, ...)
{
    static bool at_line_start = true;

    if (format == nullptr) {
        return;
    }

    char message[224] = {0};
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (written <= 0) {
        return;
    }

    if (at_line_start) {
        debug.snprint("[%010lu ms] ", static_cast<unsigned long>(HAL_GetTick()));
    }
    debug.snprint("%s", message);

    const uint32_t len = (written >= static_cast<int>(sizeof(message))) ?
            static_cast<uint32_t>(sizeof(message) - 1U) :
            static_cast<uint32_t>(written);
    if (len > 0U) {
        const char last = message[len - 1U];
        at_line_start = (last == '\n') || (last == '\r');
    }
}

#ifndef EXO_LOG
#define EXO_LOG(...) exo_log_printf(__VA_ARGS__)
#endif

#endif /* EXO_LOGGER_H_ */
