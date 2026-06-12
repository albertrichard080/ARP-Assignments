/*
 * watchdog.c - process W: signal-based watchdog
 *
 * Every active process sends SIGUSR1 to the watchdog at each cycle of
 * its main loop ("I am computing").  The watchdog records, for every
 * monitored pid, the time of the last heartbeat.  If a process stays
 * silent longer than wd_timeout seconds the watchdog sends a
 * notification (log + stderr) and then stops the whole system.
 *
 * The monitored pids are read from logs/pids.txt, written by master.
 * argv: wd_timeout
 */
#include "common.h"

#define MAXP 16

static volatile sig_atomic_t beat_pid[MAXP];
static pid_t mon_pid[MAXP];
static char  mon_name[MAXP][32];
static double last_beat[MAXP];
static int  nmon = 0;

static void on_usr1(int sig, siginfo_t *si, void *uc)
{
    (void)sig; (void)uc;
    for (int i = 0; i < nmon; i++)
        if (mon_pid[i] == si->si_pid) { beat_pid[i] = 1; return; }
}

int main(int argc, char **argv)
{
    /* install the handler before anything else: heartbeats may arrive
     * immediately and the default action of SIGUSR1 is termination */
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = on_usr1;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);
    sigset_t hb; sigemptyset(&hb); sigaddset(&hb, SIGUSR1);
    sigprocmask(SIG_UNBLOCK, &hb, NULL);   /* blocked by master before exec */

    int timeout = (argc > 1) ? atoi(argv[1]) : 10;
    FILE *lf = log_open("watchdog");
    log_line(lf, "watchdog started, pid %d, timeout %ds", getpid(), timeout);

    /* wait for master to publish the pid list */
    FILE *pf = NULL;
    for (int tries = 0; tries < 100 && nmon == 0; tries++) {
        pf = fopen(PID_FILE, "r");
        if (pf) {
            char name[32]; int pid;
            while (nmon < MAXP && fscanf(pf, "%31s %d", name, &pid) == 2) {
                /* the watchdog does not monitor itself */
                if (pid == getpid()) continue;
                strcpy(mon_name[nmon], name);
                mon_pid[nmon] = pid;
                last_beat[nmon] = now_sec();
                nmon++;
            }
            fclose(pf);
        }
        if (nmon == 0) usleep(100000);
    }
    log_line(lf, "monitoring %d processes", nmon);
    for (int i = 0; i < nmon; i++)
        log_line(lf, "  %s (pid %d)", mon_name[i], mon_pid[i]);

    for (;;) {
        sleep(1);
        double tnow = now_sec();
        for (int i = 0; i < nmon; i++) {
            if (beat_pid[i]) { beat_pid[i] = 0; last_beat[i] = tnow; }
        }
        for (int i = 0; i < nmon; i++) {
            /* master never heartbeats: it only waits; skip it */
            if (!strcmp(mon_name[i], "master")) continue;
            if (tnow - last_beat[i] > timeout) {
                log_line(lf, "NOTIFICATION: process %s (pid %d) inactive for "
                             "more than %d s - no computation going on",
                         mon_name[i], mon_pid[i], timeout);
                log_line(lf, "stopping the whole system");
                fprintf(stderr, "\n[watchdog] %s (pid %d) inactive > %ds: "
                                "stopping the system\n",
                        mon_name[i], mon_pid[i], timeout);
                for (int k = 0; k < nmon; k++) kill(mon_pid[k], SIGTERM);
                fclose(lf);
                return 1;
            }
        }
    }
}
