#ifndef ST_LOG_H__
#define ST_LOG_H__
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "common.h"

void MyLog(int log_level, const char *func_name, int line_number, const char *format, ...);
int sahara_catch_dump(int port_fd, const char *path_to_save_files, int do_reset);

#define INFO(fmt...) MyLog(LOG_INFO, __FUNCTION__, __LINE__, fmt)
#define WARN(fmt...) MyLog(LOG_WARN, __FUNCTION__, __LINE__, fmt)
#define DBG(fmt...) MyLog(LOG_DEBUG, __FUNCTION__, __LINE__, fmt)
#define ERROR(fmt...) MyLog(LOG_ERROR, __FUNCTION__, __LINE__, fmt)
#define TFTP_F "tftp:"

enum fh_log_lvl
{
    LOG_NONE, /* Disable all logs. */
    LOG_ALWAYS,
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG,
    LOG_EVENT,
    LOG_STATUS,
    LOG_FILE_ONLY, /* Put this log into the log file, not the console. */
    LOG_MAX_LIMIT
};


#endif