/*
 * master.c - creates the pipes and spawns the components with fork/exec.
 *
 * Assignment 1: Blackboard server (B), Drone dynamics (D), Keyboard
 * manager + ncurses window (I), plus the parameter file.
 * The blackboard is implemented with the process / pipes / select model.
 */
#include "common.h"

static pid_t children[4];
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
    mkdir(LOG_DIR, 0777);
    signal(SIGTERM, reap_all);
    signal(SIGINT,  reap_all);
    signal(SIGPIPE, SIG_IGN);

    /* pipes: I->B, B->I, D->B, B->D */
    int p_ib[2], p_bi[2], p_db[2], p_bd[2];
    if (pipe(p_ib) || pipe(p_bi) || pipe(p_db) || pipe(p_bd)) {
        perror("pipe"); return 1;
    }
    int allfds[8] = { p_ib[0], p_ib[1], p_bi[0], p_bi[1],
                      p_db[0], p_db[1], p_bd[0], p_bd[1] };
    int nall = 8;

    /* no watchdog in assignment 1: wd pid = 0 disables heartbeats */
    char *wds = itoa_dup(0);

    /* ---- blackboard ---- */
    {
        int keep[4] = { p_ib[0], p_bi[1], p_db[0], p_bd[1] };
        char *args[] = { "blackboard",
            itoa_dup(p_ib[0]), itoa_dup(p_bi[1]),
            itoa_dup(p_db[0]), itoa_dup(p_bd[1]), wds, NULL };
        spawn("bin/blackboard", args, allfds, nall, keep, 4);
    }
    /* ---- drone ---- */
    {
        int keep[2] = { p_bd[0], p_db[1] };
        char *args[] = { "drone", itoa_dup(p_bd[0]), itoa_dup(p_db[1]), wds, NULL };
        spawn("bin/drone", args, allfds, nall, keep, 2);
    }
    pid_t pid_I;
    /* ---- input / window (keeps the terminal) ---- */
    {
        int keep[2] = { p_ib[1], p_bi[0] };
        char *args[] = { "input", itoa_dup(p_ib[1]), itoa_dup(p_bi[0]), wds, NULL };
        pid_I = spawn("bin/input", args, allfds, nall, keep, 2);
    }

    /* master holds no pipe end: EOF must propagate between processes */
    for (int i = 0; i < nall; i++) close(allfds[i]);

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
