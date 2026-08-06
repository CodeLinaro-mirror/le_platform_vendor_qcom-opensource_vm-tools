// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear
//
// vm_powerctl.c - Send a power control command (shutdown or reboot) to a
//                 Gunyah VM via the VMM power key service. Waits for the
//                 VM process to exit.
//
// Usage:
//   vm_powerctl -v <vmid> -k <shutdown|reboot> -p <qcrosvm_pid>
//                [-t <timeout_sec>] [-h]
//
//   vm_powerctl --vmid <vmid> --key <shutdown|reboot> --pid <qcrosvm_pid>
//                [--TimeoutShutdownSec <timeout_sec>] [--help]
//
// Options:
//   -v / --vmid                VMID of the target VM (e.g. 52)
//   -k / --key                 Power control action: shutdown | reboot
//   -p / --pid                 PID of the qcrosvm main process
//   -t / --TimeoutShutdownSec  Seconds to wait for qcrosvm process exit (default: 60)
//   -h / --help                Print usage and exit
//
// Exit codes:
//   0  - Success (VM exited cleanly, or action does not terminate the VM)
//   1  - Argument validation error
//   2  - Timeout reached, qcrosvm process still running
//   3  - VMM API call failed (connect or power-key request returned an error)

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "vmm_clib.h"
#include "vmm_common.h"
#include "vmm_vmctrl.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define DEFAULT_TIMEOUT_SEC     60
#define MIN_TIMEOUT_SEC         1
#define MAX_TIMEOUT_SEC         100
#define MIN_VMID                1
#define MAX_VMID                255
#define MAX_PID                 4194304   /* PID_MAX on Linux */
#define POLL_INTERVAL_MS        500     /* 0.5 s between liveness checks */

// ---------------------------------------------------------------------------
// Power-key mapping
//   reboot            -> GVM_KEY_AGAIN  (key=2)
//   shutdown          -> GVM_KEY_STOP   (key=3)
// ---------------------------------------------------------------------------
typedef enum {
    ACTION_UNKNOWN  = 0,
    ACTION_REBOOT,
    ACTION_SHUTDOWN,
} action_t;

static const struct {
    const char *name;
    action_t    action;
} action_map[] = {
    { "reboot",   ACTION_REBOOT   },
    { "shutdown", ACTION_SHUTDOWN },
    { NULL,       ACTION_UNKNOWN  },
};

// ---------------------------------------------------------------------------
// log_kmsg() - write error-level message to /dev/kmsg (kernel ring buffer)
//
// Messages are prefixed with <3> (KERN_ERR) so they are not filtered by the
// console log level and always appear on the serial console.
//
// The motivation is PVM reboot/shutdown: by the time vm_powerctl runs,
// user-space log daemons (logd, journald) may already be dead. Writing
// directly to /dev/kmsg ensures the message enters the kernel ring buffer
// immediately, bypassing all user-space log infrastructure. During reboot
// the kernel flushes the ring buffer to the serial console, so any errors
// (e.g. timeout waiting for qcrosvm, vmm_request_gvm_pwr_key failure) are
// visible in the console log for post-mortem debugging even without a
// crash dump.
//
// g_kmsg_fd is opened once in main() and reused for all log calls to avoid
// the overhead of open/close on every message.
// ---------------------------------------------------------------------------
static int g_kmsg_fd = -1;

static void log_kmsg(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    int len;

    if (g_kmsg_fd < 0)
        return;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len < 0)
        return;

    /* prepend kernel log level <3> (KERN_ERR) and tag */
    char msg[640];
    int msg_len = snprintf(msg, sizeof(msg), "<3>vm_powerctl: %.*s\n", len, buf);
    if (msg_len < 0)
        return;
    if ((size_t)msg_len >= sizeof(msg))
        msg_len = sizeof(msg) - 1;
    (void)write(g_kmsg_fd, msg, (size_t)msg_len);
}

// ---------------------------------------------------------------------------
// usage()
// ---------------------------------------------------------------------------
static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s -v <vmid> -k <shutdown|reboot> -p <pid>"
        " [-t <timeout_sec>]\n"
        "  %s --vmid <vmid> --key <shutdown|reboot> --pid <pid>"
        " [--TimeoutShutdownSec <timeout_sec>]\n"
        "\n"
        "Options:\n"
        "  -v / --vmid                  Target VM ID (%d-%d)\n"
        "  -k / --key                   Action: shutdown | reboot\n"
        "  -p / --pid                   PID of the qcrosvm main process (1-%d)\n"
        "  -t / --TimeoutShutdownSec    Wait timeout in seconds (%d-%d, default: %d)\n"
        "  -h / --help                  Print this help message and exit\n",
        prog, prog, MIN_VMID, MAX_VMID, MAX_PID,
        MIN_TIMEOUT_SEC, MAX_TIMEOUT_SEC, DEFAULT_TIMEOUT_SEC);
}

// ---------------------------------------------------------------------------
// action_name()
// ---------------------------------------------------------------------------
static const char *action_name(action_t action)
{
    for (int i = 0; action_map[i].name != NULL; i++) {
        if (action_map[i].action == action)
            return action_map[i].name;
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// parse_action()
// ---------------------------------------------------------------------------
static action_t parse_action(const char *s)
{
    for (int i = 0; action_map[i].name != NULL; i++) {
        if (strcmp(s, action_map[i].name) == 0)
            return action_map[i].action;
    }
    return ACTION_UNKNOWN;
}

// ---------------------------------------------------------------------------
// action_to_pwr_key()
// ---------------------------------------------------------------------------
static gvm_pwr_key_t action_to_pwr_key(action_t action)
{
    switch (action) {
    case ACTION_REBOOT:   return GVM_KEY_AGAIN;
    case ACTION_SHUTDOWN: return GVM_KEY_STOP;
    default:              return _GVM_KEY_MAX;  /* invalid sentinel */
    }
}

// ---------------------------------------------------------------------------
// ms_sleep() - portable millisecond sleep
// ---------------------------------------------------------------------------
static void ms_sleep(int ms)
{
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L,
    };
    nanosleep(&ts, NULL);
}

// ---------------------------------------------------------------------------
// pid_alive() - returns 1 if process exists, 0 otherwise
// ---------------------------------------------------------------------------
static int pid_alive(pid_t pid)
{
    errno = 0;
    return (kill(pid, 0) == 0 || errno == EPERM) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// wait_for_exit() - poll until pid exits or timeout
//
// start_time: time the power key was sent, used to report elapsed time.
//
// Returns:
//   0  - process exited within timeout
//   2  - timed out, process still running
// ---------------------------------------------------------------------------
static int wait_for_vm_exit(pid_t pid, unsigned int timeout_sec, time_t start_time)
{
    unsigned int max_iters = (timeout_sec * 1000U) / POLL_INTERVAL_MS;

    if (!pid_alive(pid)) {
        log_kmsg("qcrosvm process %d already exited", (int)pid);
        return 0;
    }

    log_kmsg("waiting for qcrosvm process %d to exit (timeout=%us)", (int)pid, timeout_sec);

    for (unsigned int i = 0; i < max_iters; i++) {
        if (!pid_alive(pid)) {
            log_kmsg("qcrosvm process %d exited after %lds", (int)pid, (long)(time(NULL) - start_time));
            return 0;
        }
        ms_sleep(POLL_INTERVAL_MS);
    }

    log_kmsg("timeout (%us) reached, qcrosvm process %d still running", timeout_sec, (int)pid);

    // Dump thread status from /proc
    char path[64];
    snprintf(path, sizeof(path), "/proc/%d/task", (int)pid);
    DIR *dir = opendir(path);
    if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_name[0] == '.')
                continue;
            char status_path[128];
            snprintf(status_path, sizeof(status_path),
                     "/proc/%d/task/%s/status", (int)pid, ent->d_name);
            FILE *f = fopen(status_path, "r");
            if (f) {
                char line[256];
                char name_buf[64] = "", state_buf[64] = "";
                while (fgets(line, sizeof(line), f)) {
                    if (strncmp(line, "Name:", 5) == 0)
                        snprintf(name_buf, sizeof(name_buf), "%s", line + 5);
                    else if (strncmp(line, "State:", 6) == 0)
                        snprintf(state_buf, sizeof(state_buf), "%s", line + 6);
                }
                fclose(f);
                // strip leading whitespace and trailing newline
                char *n = name_buf;  while (*n == ' ' || *n == '\t') n++;
                char *s = state_buf; while (*s == ' ' || *s == '\t') s++;
                n[strcspn(n, "\n")] = '\0';
                s[strcspn(s, "\n")] = '\0';
                log_kmsg("  TID %-7s  Name: %-16s  State: %s", ent->d_name, n, s);
            }
        }
        closedir(dir);
    }

    return 2;
}

// ---------------------------------------------------------------------------
// args_t - parsed command-line arguments
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t     vmid;
    action_t     action;
    unsigned int timeout_sec;
    pid_t        qcrosvm_pid;
} args_t;

// ---------------------------------------------------------------------------
// parse_args() helpers
// ---------------------------------------------------------------------------

/* Return the next argv token or print an error and return NULL. */
static const char *next_arg(const char *opt, int *i, int argc, char *argv[])
{
    if (*i + 1 >= argc) {
        fprintf(stderr, "error: %s requires an argument\n", opt);
        usage(argv[0]);
        return NULL;
    }
    return argv[++(*i)];
}

/* Print a formatted error message and usage, then return 1 for use with return. */
static int arg_error(const char *prog, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    usage(prog);
    return 1;
}

// ---------------------------------------------------------------------------
// parse_args() - parse and validate argv into args_t
//
// Returns 0 on success, 1 on error (error already printed to stderr).
// ---------------------------------------------------------------------------
static int parse_args(int argc, char *argv[], args_t *args)
{
    int vmid_set   = 0;
    int action_set = 0;
    int pid_set    = 0;

    args->vmid        = 0;
    args->action      = ACTION_UNKNOWN;
    args->timeout_sec = DEFAULT_TIMEOUT_SEC;
    args->qcrosvm_pid = 0;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *val;
        char *end;

        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            usage(argv[0]);
            if (g_kmsg_fd >= 0) close(g_kmsg_fd);
            exit(0);
        } else if (strcmp(arg, "-v") == 0 || strcmp(arg, "--vmid") == 0) {
            if (!(val = next_arg(arg, &i, argc, argv))) return 1;
            errno = 0;
            unsigned long v = strtoul(val, &end, 10);
            if (errno != 0 || *end != '\0' || v < MIN_VMID || v > MAX_VMID)
                return arg_error(argv[0],
                    "error: invalid vmid '%s' (must be %d-%d)\n",
                    val, MIN_VMID, MAX_VMID);
            args->vmid = (uint32_t)v;
            vmid_set = 1;
        } else if (strcmp(arg, "-k") == 0 || strcmp(arg, "--key") == 0) {
            if (!(val = next_arg(arg, &i, argc, argv))) return 1;
            args->action = parse_action(val);
            if (args->action == ACTION_UNKNOWN)
                return arg_error(argv[0], "error: invalid key '%s' (must be shutdown|reboot)\n", val);
            action_set = 1;
        } else if (strcmp(arg, "-t") == 0 || strcmp(arg, "--TimeoutShutdownSec") == 0) {
            if (!(val = next_arg(arg, &i, argc, argv))) return 1;
            errno = 0;
            unsigned long t = strtoul(val, &end, 10);
            if (*end == 's') end++;
            if (errno != 0 || *end != '\0' || t < MIN_TIMEOUT_SEC || t > MAX_TIMEOUT_SEC)
                return arg_error(argv[0],
                    "error: invalid timeout '%s' (must be %d-%d)\n",
                    val, MIN_TIMEOUT_SEC, MAX_TIMEOUT_SEC);
            args->timeout_sec = (unsigned int)t;
        } else if (strcmp(arg, "-p") == 0 || strcmp(arg, "--pid") == 0) {
            if (!(val = next_arg(arg, &i, argc, argv))) return 1;
            errno = 0;
            long p = strtol(val, &end, 10);
            if (errno != 0 || *end != '\0' || p <= 0 || p > MAX_PID)
                return arg_error(argv[0],
                    "error: invalid pid '%s' (must be 1-%d)\n",
                    val, MAX_PID);
            args->qcrosvm_pid = (pid_t)p;
            pid_set = 1;
        } else {
            return arg_error(argv[0], "error: unrecognized option '%s'\n", arg);
        }
    }

    if (!vmid_set)
        return arg_error(argv[0], "error: --vmid / -v is required\n");
    if (!action_set)
        return arg_error(argv[0], "error: --key / -k is required\n");
    if (!pid_set)
        return arg_error(argv[0], "error: --pid / -p is required\n");

    return 0;
}

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    args_t args;
    int    ret = 0;

    g_kmsg_fd = open("/dev/kmsg", O_WRONLY | O_CLOEXEC);

    if (parse_args(argc, argv, &args) != 0) {
        ret = 1;
        goto out;
    }

    gvm_pwr_key_t pwr_key = action_to_pwr_key(args.action);

    log_kmsg("invoked: vmid=%u action=%s pwr_key=%d timeout=%us pid=%d",
             args.vmid, action_name(args.action), (int)pwr_key,
             args.timeout_sec, (int)args.qcrosvm_pid);

    // -----------------------------------------------------------------------
    // Step 1: Connect to VMM power-manager and send the VMM power-key event
    // -----------------------------------------------------------------------
    time_t start_time = time(NULL);
    void *pwr_handle = NULL;
    ret = vmm_client_connect("vm_powerctl", VMM_POWER_MANAGER_SERVER, &pwr_handle);
    if (ret != (int)EOK) {
        log_kmsg("error: failed to connect to VMM power manager (ret=%d)", ret);
        ret = 3;
        goto out;
    }

    ret = vmm_request_gvm_pwr_key(args.vmid, pwr_key, pwr_handle);
    if (ret != (int)EOK) {
        log_kmsg("error: vmm_request_gvm_pwr_key failed (ret=%d)", ret);
        (void)vmm_client_disconnect(pwr_handle);
        ret = 3;
        goto out;
    }

    log_kmsg("vmm_request_gvm_pwr_key succeeded for vmid=%u key=%d",
             args.vmid, (int)pwr_key);
    (void)vmm_client_disconnect(pwr_handle);

    // -----------------------------------------------------------------------
    // Step 2: Wait for qcrosvm main process to exit
    // -----------------------------------------------------------------------
    ret = wait_for_vm_exit(args.qcrosvm_pid, args.timeout_sec, start_time);

out:
    if (g_kmsg_fd >= 0)
        close(g_kmsg_fd);

    return ret;
}
