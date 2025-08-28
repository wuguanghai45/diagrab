#include "log.h"

#define LOG_BUFFER_SIZE 2048
void MyLog(int log_level, const char *func_name, int line_number, const char *format, ...)
{
    va_list args;
    char log[LOG_BUFFER_SIZE];

    time_t current_time;
    struct tm *local_time;

    current_time = time(NULL);
    local_time = localtime(&current_time);

    va_start(args, format);
    vsnprintf(log, sizeof(log), format, args);
    va_end(args);

    printf("\n[%02d:%02d:%02d] ", local_time->tm_hour, local_time->tm_min, local_time->tm_sec);

    switch (log_level){
    case LOG_ERROR:
        printf("ERROR: function: %s:%d %s\n", func_name, line_number, log);
        break;

    case LOG_INFO:
        // printf("INFO: function: %s:%d %s\n", func_name, line_number, log);
        printf("INFO: %s", log);
        break;

    case LOG_DEBUG:
        // printf("DEBUG: function: %s:%d %s\n", func_name, line_number, log);
        printf("DEBUG: %s", log);
        break;

    case LOG_WARN:
        printf("WARNING: function: %s:%d %s\n", func_name, line_number, log);
        break;

    case LOG_EVENT:
        printf("EVENT: %s\n", log);
        break;

    case LOG_STATUS:
        printf("%s\n", log);
        break;
    }
}
