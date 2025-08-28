#include <malloc.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include "common.h"
#include "log.h"

#define PROGRAM_VERSION "0.0.3" "(20210521)"

char diag_port_name[32] = "/dev/ttyUSB0";
//const char *diag_output_dir = "qxdmlog";
const char *tftp_server = ".";
char diag_output_dir[256];
const char *mask_cfg_file = NULL;
unsigned long max_file_size = 100000000;
#define LOGFILE_MAX_NUM 512
static int diag_fd = -1;
static int mask_reads_succeeded = 0;
smallint ss_got_signal = 0;
int use_qmdl2_v2 = 1;
int use_ram_dump = 0;
int use_tftp = 0;
#define QUALCOMM_QXDM_LOG_BUFF_SIZE (16 * 1024)
static char s_logfile_List[512][32];
static unsigned s_logfile_num = 0;
static unsigned s_logfile_seq;
static unsigned s_logfile_idx;
static int mask_file_external_entered = 0;
static unsigned exit_after_usb_disconnet = 0;

unsigned clock_gettime_msecs(void)
{
    static unsigned start = 0;
    unsigned now;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = (unsigned)ts.tv_sec * 1000 + (unsigned)(ts.tv_nsec / 1000000);
    if (start == 0)
        start = now;
    return now - start;
}

void ClosePort(void)
{
    if (-1 == diag_fd)
        return;
    if (diag_fd != -1)
    {
        DBG("Disconnecting from com port %s\n", diag_port_name);
        close(diag_fd);
        diag_fd = -1;
    }
}


ssize_t full_poll_read_fds(int *fds, int n, void *pbuf, size_t size, unsigned timeout_msec)
{
    unsigned t;
    ssize_t rc = 0;

    while (ss_got_signal == 0 && timeout_msec > 0)
    {
        struct pollfd pollfds[4];
        int ret = -1;
        int i = 0;

        for (i = 0; i < n; i++)
        {
            pollfds[i].events = POLLIN;
            pollfds[i].fd = fds[i];
        }

        t = (timeout_msec < 200) ? timeout_msec : 200;
        ret = poll(pollfds, n, t);

        if (ret == 0)
        {
            timeout_msec -= t;
            if (timeout_msec == 0)
            {
                rc = -1;
                errno = ETIMEDOUT;
                break;
            }
            continue;
        }
        else if (ret < 0 && errno == EINTR)
        {
            continue;
        }
        else if (ret < 0)
        {
            DBG("poll(handlefd) =%d, errno: %d (%s)", ret, errno, strerror(errno));
            break;
        }

        for (i = 0; i < n; i++)
        {
            if (pollfds[i].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                DBG("poll fd=%d, revents = %04x", pollfds[i].fd, pollfds[i].revents);
                goto _out;
            }

            if (pollfds[i].revents & (POLLIN))
            {
                rc = read(pollfds[i].fd, pbuf, size);
                fds[0] = pollfds[i].fd;
                goto _out;
            }
        }
    }

_out:
    return rc;
}

ssize_t safe_poll_read(int fd,  void *pbuf, size_t size, unsigned timeout_msec) {
    return full_poll_read_fds(&fd, 1, pbuf, size, timeout_msec);
}

ssize_t safe_poll_write(int fd, const void *buf, size_t size, unsigned timeout_msec)
{
    size_t wc = 0;
    ssize_t nbytes;

    nbytes = write(fd, buf + wc, size - wc);

    if (nbytes <= 0)
    {
        if (errno != EAGAIN)
        {
            DBG("Fail to write fd = %d, errno : %d (%s)", fd, errno, strerror(errno));
            goto out;
        }
        else
        {
            nbytes = 0;
        }
    }

    wc += nbytes;

    while (wc < size)
    {
        int ret;
        struct pollfd pollfds[] = {{fd, POLLOUT, 0}};

        ret = poll(pollfds, 1, timeout_msec);

        if (ret <= 0)
        {
            DBG("Fail to poll fd = %d, errno : %d (%s)", fd, errno, strerror(errno));
            break;
        }

        if (pollfds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            DBG("Fail to poll fd = %d, revents = %04x", fd, pollfds[0].revents);
            break;
        }

        if (pollfds[0].revents & (POLLOUT))
        {
            nbytes = write(fd, buf + wc, size - wc);

            if (nbytes <= 0)
            {
                DBG("Fail to write fd = %d, errno : %d (%s)", fd, errno, strerror(errno));
                break;
            }
            wc += nbytes;
        }
    }

out:
    if (wc != size)
    {
        DBG("%s fd=%d, size=%zd, timeout=%d, wc=%zd", __func__, fd, size, timeout_msec, wc);
    }

    return (wc);
}

size_t get_qsr4_db_filesize(const char *filename)
{
    char fullname[255 + 2];
    int fd = -1;
    size_t file_len = 0;

    if (diag_output_dir == NULL || *diag_output_dir == '\0')
    {
        return -1;
    }

    sprintf(fullname, "%s/%s", diag_output_dir, filename);

    fd = open(fullname, O_RDONLY);
    if (fd != -1)
    {
        file_len = lseek(fd, 0, SEEK_END);
        close(fd);
    }

    return file_len;
}

int create_qsr4_db_file(const char *filename)
{
    char fullname[255 + 2];
    int fd = -1;

    if (diag_output_dir == NULL || *diag_output_dir == '\0')
    {
        return -1;
    }

    sprintf(fullname, "%s/%s", diag_output_dir, filename);

    fd = open(fullname, O_CREAT | O_WRONLY | O_TRUNC, 0444);
    if (fd <= 0)
    {
        DBG("Fail to create %s! errno : %d (%s)", fullname, errno, strerror(errno));
    }

    return fd;
}

static int create_log_file(const char *logfile_dir, const char *logfile_suffix, unsigned logfile_seq)
{
    int logfd;
    char shortname[32];
    char filename[255 + 1];

    if (s_logfile_num && s_logfile_List[logfile_seq % s_logfile_num][0])
    {
        sprintf(filename, "%s/%s.%s", logfile_dir, s_logfile_List[logfile_seq % s_logfile_num], logfile_suffix);
        if (access(filename, R_OK) == 0)
        {
            remove(filename);
        }
    }

    time_t ltime;
    struct tm *currtime;

    time(&ltime);
    currtime = localtime(&ltime);

    snprintf(shortname, sizeof(shortname), "%04d%02d%02d_%02d%02d%02d_%04d",
             (currtime->tm_year + 1900), (currtime->tm_mon + 1), currtime->tm_mday,
             currtime->tm_hour, currtime->tm_min, currtime->tm_sec, logfile_seq);
    sprintf(filename, "%s/%s.%s", logfile_dir, shortname, logfile_suffix);

    logfd = open(filename, O_CREAT | O_WRONLY | O_TRUNC, 0444);
    if (logfd <= 0)
    {
        DBG("Fail to create new logfile! errno : %d (%s)", errno, strerror(errno));
    }

    DBG("__func__:%s %s logfd=%d", __func__, filename, logfd);

    if (s_logfile_num)
    {
        s_logfile_idx = (logfile_seq % s_logfile_num);
        strcpy(s_logfile_List[s_logfile_idx], shortname);
    }

    return logfd;
}

static int close_log_file(int logfd)
{
    return close(logfd);
}

static void *init_filter_config_trd(void *arg)
{
    void **thread_args = (void **)arg;
    int *ttyfd = (int *)thread_args[0];
    const char *filter_cfg = (const char *)thread_args[1];

    qcom_init_filter(*ttyfd, filter_cfg);

    DBG("log_mask_cfg_load_done!");
    mask_reads_succeeded = 1;
    return NULL;
}

static int qualcomm_serial_logstart(int handlefd, const char *logfile_dir, size_t logfile_size, const char *filter_cfg)
{
    size_t savelog_size = 0;
    uint8_t *rbuf;
    const size_t rbuf_size = QUALCOMM_QXDM_LOG_BUFF_SIZE;
    static int logfd = -1;
    const char *logfile_suffix = "qmdl";
    pthread_t thread_id1;
    pthread_attr_t thread_attr;
    const void *thread_args[3];
    size_t total_read = 0;
    unsigned long now_msec = 0;

    if (use_qmdl2_v2)
    {
        logfile_suffix = "qmdl2";
    }

    rbuf = (uint8_t *)malloc(rbuf_size);
    if (rbuf == NULL)
    {
        DBG("Fail to malloc rbuf_size=%zd, errno: %d (%s)", rbuf_size, errno, strerror(errno));
        return -1;
    }

    thread_args[0] = &handlefd;
    thread_args[1] = filter_cfg;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&thread_id1, &thread_attr, init_filter_config_trd, (void *)thread_args);

    now_msec = clock_gettime_msecs();
    while (ss_got_signal == 0)
    {
        ssize_t rc, wc;
        unsigned long n;
        int fds[2] = {handlefd};

        rc = full_poll_read_fds(fds, 1, rbuf, rbuf_size, -1);
        if (rc <= 0)
        {
            break;
        }

        n = clock_gettime_msecs();
        total_read += rc;

        if ((total_read >= (16 * 1024 * 1024)) || (n >= (now_msec + 3000)))
        {
            DBG("recv: %zdM %zdK %zdB  in %ld msec", total_read / (1024 * 1024),
                total_read / 1024 % 1024, total_read % 1024, n - now_msec);
            now_msec = n;
            total_read = 0;
        }

        if (logfd == -1)
        {
            logfd = create_log_file(logfile_dir, logfile_suffix, s_logfile_seq);
            if (logfd <= 0)
            {
                break;
            }
            {
                qcom_logfile_init(logfd);
                s_logfile_seq++;
            }
        }

        wc = qcom_logfile_save(logfd, rbuf, rc);

        if (wc != rc)
        {
            DBG("savelog fail %zd/%zd, break", wc, rc);
            exit_after_usb_disconnet = 1; //Maybe no memory so exit diaggrab
            break;
        }

        savelog_size += wc;

        if (savelog_size >= logfile_size)
        {
            savelog_size = 0;
            close_log_file(logfd);
            logfd = -1;
        }
    }

    if (logfd > 0)
        close_log_file(logfd);
    logfd = -1;
    free(rbuf);

    if (ss_got_signal)
        send_empty_mask();

    return 0;
}

static void sig_handler(int signal_num)
{
    DBG("recv signal %d\n", signal_num);
    ss_got_signal = 1;
}

static void usage(char *progname)
{
    printf("\n Usage for %s:\n", progname);
    printf("\n-h  --help:\t usage help\n");
    printf("\n-p  --port:\t TTY device to use. Example /dev/ttyUSB0\n");
    printf("\n-s  --size:\t maximum file size in MB, default is 100\n");
    printf("\n-n  --lognum:\t maximum file num[0-512], default is 0. 0 means no limit.\n");
    printf("\n-c  --filemdm:\t mask file name for MDM\n");
    printf("\n-u, --qmdl2_v2:\t Guid-diagid mapping in qmdl2 header\n");
    printf("\n-r, --ramdump:\t catch SDX55 ram dump\n");

    exit(0);
}


#define default_options (options_t){ {'\0'}, NULL}
typedef struct {
    char ttyport[32];
    struct usb_ifc_info *usb_info;
} options_t;

static void parse_args(int argc, char **argv, options_t *opt)
{
    int command;
    int file_num = 0;
    struct option longopts[] =
        {
            {"help", 0, NULL, 'h'},
            {"port", 1, NULL, 'p'},
            {"filemdm", 1, NULL, 'c'},
            {"output", 1, NULL, 'o'},
            {"size", 1, NULL, 's'},
            {"number", 1, NULL, 'n'},
            {"qmdl2_v2", 0, NULL, 'u'},
            {"ramdump", 0, NULL, 'r'},
            {"tftp", 1, NULL, 't'},
            {"abort", 0, NULL, 'q'},
        };

    while ((command = getopt_long(argc, argv, "o:p:c:s:n:t:hurq", longopts, NULL)) != -1)
    {
        switch (command)
        {
        case 'p':
            snprintf(opt->ttyport, sizeof(opt->ttyport), "%s", optarg);
            DBG("Diag Port name: '%s'", opt->ttyport);
            break;
        case 'c':
            mask_cfg_file = optarg;
            DBG("the mask file: '%s'", mask_cfg_file);
            mask_file_external_entered = 1;
            break;
        case 'n':
            file_num = atoi(optarg);
            if (file_num <= 1)
            {
                ERROR("Invalid file number, must be greater than 1");
                exit(0);
            }
            if (file_num > LOGFILE_MAX_NUM)
            {
                file_num = LOGFILE_MAX_NUM;
            }

            s_logfile_num = file_num;
            break;
        case 's':
            max_file_size = atol(optarg);
            if ((long)max_file_size <= 0)
                max_file_size = 100000000;
            else
            {
                max_file_size *= 1024 * 1024;
                if (max_file_size > 0 && max_file_size < 1024 * 1024)
                    max_file_size = 100000000;
            }
            break;
        case 'u':
            use_qmdl2_v2 = 1;
            break;
        case 'r': //ram dump
            use_ram_dump = 1;
            break;
        case 'q':
            exit_after_usb_disconnet = 1; //abort diaggrab otherwise keep running
            break;
        case 't':
            tftp_server = optarg;
            use_tftp = 1;
            DBG("tftp_server %s",tftp_server);
            break;
        case 'h':
        default:
            usage(argv[0]);
        };
    }
    DBG("The given mask file: %s", mask_cfg_file ? mask_cfg_file : "Built-in mask");
}

static int serial_open(const char *device)
{
    struct termios tio;

    if (!device)
    {
        DBG("invalid parameter!");
        goto error_exit;
    }

    DBG("User wants to talk to port '%s'", device);

    int ttyfd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ttyfd < 0)
    {
        DBG("Fail to open %s, errno[%d] (%s)", device, errno, strerror(errno));
        goto error_exit;
    }
    fcntl(ttyfd, F_SETFL, O_RDWR);

    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(ttyfd, &tio) == -1)
    {
        ERROR("can't tcgetattr for %s", device);
        goto error_exit;
    };
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    if (tcsetattr(ttyfd, TCSANOW, &tio) == -1)
    {
        ERROR("can't tcsetattr for %s errno(%d)", device, errno);
        goto error_exit;
    }

    diag_fd = ttyfd;
    return 0;

error_exit:
    if (diag_fd != -1)
    {
        close(diag_fd);
        diag_fd = -1;
    }

    return -1;
}
static const char sys_bus_usb_devices[] = "/sys/bus/usb/devices";
typedef struct usb_ifc_info usb_ifc_info;

#define MAX_PATH 1024
#define SYSFS_INTu(de, tgt, name) do { tgt.name = read_sysfs_file_int(de,#name,10); } while(0)
#define SYSFS_INTx(de, tgt, name) do { tgt.name = read_sysfs_file_int(de,#name,16); } while(0)


struct usb_ifc_info
{
    unsigned short idVendor;
    unsigned short idProduct;
    unsigned char devnum;  /* Device address */
    unsigned short busnum; /* Bus number */
    unsigned int bcdDevice;
    unsigned int bInterfaceClass;
    unsigned int bNumInterfaces;
    char ttyDM[32];
};

static struct usb_ifc_info s_usb_device_info[8];

static unsigned int read_sysfs_file_int(const char *d_name, const char *file, int base)
{
    char buf[12], path[MAX_PATH];
    int fd;
    ssize_t r;
    unsigned long ret;
    snprintf(path, MAX_PATH, "%s/%s/%s", sys_bus_usb_devices, d_name, file);
    path[MAX_PATH - 1] = '\0';
    fd = open(path, O_RDONLY);
    if (fd < 0)
        goto error;
    memset(buf, 0, sizeof(buf));
    r = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (r < 0)
        goto error;
    buf[sizeof(buf) - 1] = '\0';
    ret = strtoul(buf, NULL, base);
    return (unsigned int)ret;

error:
    perror(path);
    return 0;
}

static inline int badname(const char *name)
{
    if (!isdigit(*name))
        return 1;
    while (*++name)
    {
        if (!isdigit(*name) && *name != '.' && *name != '-')
            return 1;
    }
    return 0;
}

static int find_matched_device(void)
{
    struct dirent *de, *ttyde;
    DIR *busdir = NULL, *dev_dir = NULL;
    struct usb_ifc_info info;
    int wait_tty_register, modules_num = 0;

    busdir = opendir(sys_bus_usb_devices);
    if(busdir == NULL) return 0;

    while ((de = readdir(busdir))) {
        char devpath[256*2] = {'\0'};
        if(badname(de->d_name)) continue;

        memset(&info, 0, sizeof(struct usb_ifc_info));

        SYSFS_INTx(de->d_name, info, idVendor);
        SYSFS_INTx(de->d_name, info, idProduct);
        SYSFS_INTu(de->d_name, info, busnum);
        SYSFS_INTu(de->d_name, info, devnum);
        SYSFS_INTx(de->d_name, info, bcdDevice);
        SYSFS_INTx(de->d_name, info, bNumInterfaces);

        if (info.idVendor != 0x1e0e && info.idVendor != 0x05c6) continue;

        wait_tty_register = 10;
_scan_tty:
    snprintf(devpath, sizeof(devpath), "%s/%s/%s:1.0", sys_bus_usb_devices,
    de->d_name, de->d_name);
    if (access(devpath, F_OK))
        continue;

    // find tty device
    dev_dir = opendir(devpath);
    if(dev_dir){
        while ((ttyde = readdir(dev_dir)))
        {
            if (ttyde->d_name[0] == '.')
                continue;

            if (!strncasecmp(ttyde->d_name, "ttyUSB", 6))
            {
                snprintf(info.ttyDM, sizeof(info.ttyDM), "/dev/%.24s", ttyde->d_name);
                break;
            }
        }
        closedir(dev_dir);

        if (info.ttyDM[0] == '\0' && wait_tty_register)
        {
            usleep(100 * 1000);
            wait_tty_register--;
            goto _scan_tty;
        }
    }

    DBG("Find ID %x:%x bcdDevice=%4x bNumInterfaces=%03d diagport=%s",
        info.idVendor, info.idProduct, info.bcdDevice, info.bNumInterfaces, info.ttyDM);
    if (modules_num < 8)
        s_usb_device_info[modules_num++] = info;
    }

    closedir(busdir);
    return modules_num;
}

int main(int argc, char **argv)
{
    int ret = -1;
    struct sigaction sact;
    int modules_num = 0;
    int cur_module = 0;

    INFO("Build Version: %s", PROGRAM_VERSION);
    INFO("Binary build date: %s @ %s", __DATE__, __TIME__);

    options_t options = default_options;

    parse_args(argc, argv, &options);

    sigemptyset(&sact.sa_mask);
    sact.sa_flags = 0;
    sact.sa_handler = sig_handler;
    sigaction(SIGTERM, &sact, NULL);
    sigaction(SIGHUP, &sact, NULL);
    sigaction(SIGINT, &sact, NULL);

__main_loop:
    if (ss_got_signal)
    {
        return EXIT_SUCCESS;
    }

    diag_fd = -1;
    options.usb_info = NULL;
    memset(s_usb_device_info, 0, sizeof(s_usb_device_info));
    cur_module = modules_num = 0;

    if (options.ttyport && !strncmp(options.ttyport, "/dev/mhi", strlen("/dev/mhi")))
    {
        struct usb_ifc_info *mhi_dev = &s_usb_device_info[0];
        memset(mhi_dev, 0, sizeof(struct usb_ifc_info));
        mhi_dev->bNumInterfaces = 6;
        strncpy(mhi_dev->ttyDM, options.ttyport, sizeof(mhi_dev->ttyDM));
        modules_num = 1;
        exit_after_usb_disconnet = 1;
    }

    if (modules_num == 0)
    {
        modules_num = find_matched_device();
        if (modules_num == 0)
        {
            DBG("Wait for modules connect or Press CTRL+C to quit!");
            sleep(2);
            goto __main_loop;
        }
    }

    if (options.ttyport && !strncmp(options.ttyport, "/dev/ttyUSB", strlen("/dev/ttyUSB")))
    {
        for (cur_module = 0; cur_module < modules_num; cur_module++)
        {
            if (!strcmp(options.ttyport, s_usb_device_info[cur_module].ttyDM))
            {
                break;
            }
        }
        if (cur_module == modules_num)
        {
            DBG("No %s find, wait for connect!", options.ttyport);
            goto __main_loop;
        }
    }

    options.usb_info = &s_usb_device_info[cur_module];
    snprintf(options.ttyport, sizeof(options.ttyport), "%s", options.usb_info->ttyDM);

    if (options.ttyport[0] == '\0')
    {
        goto __main_loop;
    }
    DBG("Currently bcdDevice %4x",options.usb_info->bcdDevice);
    if (options.usb_info->bcdDevice == le32_to_cpu(0x0318))
    {
        use_qmdl2_v2 = 0;
    }

    ret = serial_open(options.ttyport);
    if (ret < 0)
    {
        ret = -ret;
        goto done;
    }

    if (diag_fd < 0)
    {
        DBG("tty open %s failed, errno: %d (%s)", diag_port_name, errno, strerror(errno));
        goto done;
    }

    time_t ltime;
    struct tm *currtime;

    time(&ltime);
    currtime = localtime(&ltime);
    snprintf(diag_output_dir, sizeof(diag_output_dir),
              "Qxdmlog_%04d%02d%02d_%02d%02d%02d",
             (currtime->tm_year + 1900), (currtime->tm_mon + 1), currtime->tm_mday,
             currtime->tm_hour, currtime->tm_min, currtime->tm_sec);

    if (access(diag_output_dir, F_OK) && errno == ENOENT)
        mkdir(diag_output_dir, 0755);

    if (use_ram_dump || s_usb_device_info[cur_module].bNumInterfaces == 1)
    {
        DBG("catch ramdump\n");
        if (use_tftp) {
            ret = sahara_catch_dump(diag_fd, tftp_server, 1);
        }else {
            ret = sahara_catch_dump(diag_fd, diag_output_dir, 1);
        }
    }else{
        DBG("Press CTRL+C to stop catch log.");
        ret = qualcomm_serial_logstart(diag_fd, diag_output_dir, max_file_size, mask_cfg_file);
    }

done:
    ClosePort();

    if (ss_got_signal == 0 && exit_after_usb_disconnet == 0)
    {
        sleep(1);
        goto __main_loop;
    }

    return ret;
}
