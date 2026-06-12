/*
 * master.c - creates the pipes, spawns every component with fork/exec,
 * publishes the pid list for the watchdog and supervises termination.
 *
 * Assignment 2: full system - blackboard (B), drone (D), keyboard +
 * ncurses windows (I), obstacle generator (O), target generator (T),
 * signal-based watchdog (W), parameter file and per-process logfiles.
 */
#include "common.h"

static pid_t children[8];
static int nchildren = 0;

static void reap_all(int sig)
{
    (void)sig;
    for (int i = 0; i < nchildren; i++)
        if (children[i] > 0) kill(children[i], SIGTERM);
    while (waitpid(-1, NULL, WNOHANG) > 0) ;
    _exit(0);
}

/* fork+exec `path`; close every pipe fd the child does not need */
static pid_t spawn(const char *path, char *const argv[],
                   int *allfds, int nall, int *keep, int nkeep)
{
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(1); }
    if (pid == 0) {
        for (int i = 0; i < nall; i++) {
            int needed = 0;
            for (int k = 0; k < nkeep; k++) if (allfds[i] == keep[k]) needed = 1;
            if (!needed) close(allfds[i]);
        }
        execv(path, argv);
        perror(path);
        _exit(127);
    }
    children[nchildren++] = pid;
    return pid;
}

static char *itoa_dup(int v) { char b[16]; snprintf(b, sizeof b, "%d", v); return strdup(b); }

int main(void)
{
    params_t P; params_default(&P); params_load(PARAM_FILE, &P);

    mkdir(LOG_DIR, 0777);
    signal(SIGTERM, reap_all);
    signal(SIGINT,  reap_all);
    signal(SIGPIPE, SIG_IGN);

    /* block SIGUSR1 before forking: the mask is inherited across fork and
     * exec, so no child can be killed by an early heartbeat while the
     * watchdog has not installed its handler yet (the watchdog unblocks
     * the signal after sigaction) */
    sigset_t hb; sigemptyset(&hb); sigaddset(&hb, SIGUSR1);
    sigprocmask(SIG_BLOCK, &hb, NULL);

    /* pipes: [0]=read end, [1]=write end */
    int p_ib[2], p_bi[2], p_db[2], p_bd[2], p_ob[2], p_tb[2], p_bt[2];
    if (pipe(p_ib) || pipe(p_bi) || pipe(p_db) || pipe(p_bd) ||
        pipe(p_ob) || pipe(p_tb) || pipe(p_bt)) { perror("pipe"); return 1; }

    int allfds[14] = { p_ib[0], p_ib[1], p_bi[0], p_bi[1], p_db[0], p_db[1],
                       p_bd[0], p_bd[1], p_ob[0], p_ob[1], p_tb[0], p_tb[1],
                       p_bt[0], p_bt[1] };
    int nall = 14;

    /* ---- watchdog first, so its pid is known to everybody ---- */
    char *wargs[] = { "watchdog", itoa_dup(P.wd_timeout), NULL };
    pid_t wd = spawn("bin/watchdog", wargs, allfds, nall, NULL, 0);
    char *wds = itoa_dup((int)wd);

    /* ---- blackboard ---- */
    {
        int keep[7] = { p_ib[0], p_bi[1], p_db[0], p_bd[1],
                        p_ob[0], p_tb[0], p_bt[1] };
        char *args[] = { "blackboard",
            itoa_dup(p_ib[0]), itoa_dup(p_bi[1]),
            itoa_dup(p_db[0]), itoa_dup(p_bd[1]),
            itoa_dup(p_ob[0]),
            itoa_dup(p_tb[0]), itoa_dup(p_bt[1]),
            wds, NULL };
        spawn("bin/blackboard", args, allfds, nall, keep, 7);
    }
    /* ---- drone ---- */
    {
        int keep[2] = { p_bd[0], p_db[1] };
        char *args[] = { "drone", itoa_dup(p_bd[0]), itoa_dup(p_db[1]), wds, NULL };
        spawn("bin/drone", args, allfds, nall, keep, 2);
    }
    pid_t pid_I;
    /* ---- input / windows (keeps the terminal) ---- */
    {
        int keep[2] = { p_ib[1], p_bi[0] };
        char *args[] = { "input", itoa_dup(p_ib[1]), itoa_dup(p_bi[0]), wds, NULL };
        pid_I = spawn("bin/input", args, allfds, nall, keep, 2);
    }
    /* ---- obstacle generator ---- */
    {
        int keep[1] = { p_ob[1] };
        char *args[] = { "obstacles", itoa_dup(p_ob[1]), wds, NULL };
        spawn("bin/obstacles", args, allfds, nall, keep, 1);
    }
    /* ---- target generator ---- */
    {
        int keep[2] = { p_tb[1], p_bt[0] };
        char *args[] = { "targets", itoa_dup(p_tb[1]), itoa_dup(p_bt[0]), wds, NULL };
        spawn("bin/targets", args, allfds, nall, keep, 2);
    }

    /* master holds no pipe end: EOF must propagate between processes */
    for (int i = 0; i < nall; i++) close(allfds[i]);

    /* publish the pid list for the watchdog */
    FILE *pf = fopen(PID_FILE, "w");
    if (pf) {
        const char *names[] = { "watchdog", "blackboard", "drone",
                                "input", "obstacles", "targets" };
        for (int i = 0; i < nchildren; i++)
            fprintf(pf, "%s %d\n", names[i], children[i]);
        fprintf(pf, "master %d\n", getpid());
        fclose(pf);
    }

    /* wait until the user interface terminates, then stop everything */
    for (;;) {
        int st;
        pid_t done = waitpid(-1, &st, 0);
        if (done < 0) break;
        if (done == pid_I) break;
    }
    usleep(300000);
    for (int i = 0; i < nchildren; i++)
        if (children[i] > 0) kill(children[i], SIGTERM);
    while (waitpid(-1, NULL, 0) > 0) ;
    printf("simulation ended - per-process logs are in ./logs/\n");
    return 0;
}
