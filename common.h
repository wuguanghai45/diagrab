

#ifndef __COMDEF_H__
#define __COMDEF_H__

#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>
#include <pthread.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include "utils.h"
#include "log.h"

typedef signed char smallint;
typedef int8_t        int8;
typedef int16_t       int16;
typedef int32_t       int32;
typedef int64_t       int64;
typedef uint8_t       uint8;
typedef uint16_t      uint16;
typedef uint32_t      uint32;
typedef uint64_t      uint64;
typedef int8_t        INT8;
typedef int16_t       INT16;
typedef int32_t       INT32;
typedef int64_t       INT64;
typedef uint8_t       UINT8;
typedef uint16_t      UINT16;
typedef uint32_t      UINT32;
typedef uint64_t      UINT64;

extern smallint ss_got_signal;
extern int use_qmdl2_v2;
extern size_t get_qsr4_db_filesize(const char *filename);
extern int create_qsr4_db_file(const char *filename);
extern size_t qcom_logfile_save(int logfd, const void *buf, size_t size);
extern int qcom_logfile_init(int logfd);
extern int qcom_init_filter(int fd, const char *cfg);
extern ssize_t qcom_send_cmd(int fd, const unsigned char *buf, size_t size);
extern void send_empty_mask();
extern unsigned clock_gettime_msecs(void);
extern ssize_t safe_poll_write(int fd, const void *buf, size_t size, unsigned timeout_mesc);
extern ssize_t safe_poll_read(int fd, void *pbuf, size_t size, unsigned timeout_msec);

#endif /* __COMDEF_H__ */