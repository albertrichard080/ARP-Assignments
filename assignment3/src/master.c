/*
 * master.c - creates the pipes, spawns every component with fork/exec,
 * publishes the pid list for the watchdog and supervises termination.
 *
 * Assignment 3 modes (chosen before start, as allowed by the spec):
 *      ./bin/master                 -> asks standalone / server / client
 *      ./bin/master standalone
 *      ./bin/master server
 *      ./bin/master client [addr]
 *
 * In server and client mode the obstacle generator, the target
 * generator and the watchdog are NOT started (turned off, as required),
 * and the network process N is started instead.
 */
#include "common.h"

#define MODE_STANDALONE 0
#define MODE_SERVER     1
#define MODE_CLIENT     2

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

/* fork+exec `path` with NULL-terminated string args; close every pipe fd
 * the child does not need (keep[] holds the fds it must inherit) */
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

int main(int argc, char **argv)
{
    int mode = -1;
    char addr[64] = "";

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);
    strcpy(addr, P.net_addr);

    if (argc > 1) {
        if      (!strcmp(argv[1], "standalone")) mode = MODE_STANDALONE;
        else if (!strcmp(argv[1], "server"))     mode = MODE_SERVER;
        else if (!strcmp(argv[1], "client"))     mode = MODE_CLIENT;
        if (mode == MODE_CLIENT && argc > 2) { strncpy(addr, argv[2], 63); addr[63] = 0; }
    }
    if (mode < 0) {                       /* ask the user before starting */
        char ans[32];
        printf("Run local (standalone) or networked? [l/n]: ");
        fflush(stdout);
        if (!fgets(ans, sizeof ans, stdin)) return 1;
        if (ans[0] == 'n' || ans[0] == 'N') {
            printf("Act as server or client? [s/c]: ");
            fflush(stdout);
            if (!fgets(ans, sizeof ans, stdin)) return 1;
            mode = (ans[0] == 'c' || ans[0] == 'C') ? MODE_CLIENT : MODE_SERVER;
            if (mode == MODE_CLIENT) {
                printf("Server address [%s]: ", addr);
                fflush(stdout);
                if (fgets(ans, sizeof ans, stdin) && ans[0] != '\n')
                    sscanf(ans, "%63s", addr);
            }
        } else mode = MODE_STANDALONE;
    }

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

    int local = (mode == MODE_STANDALONE);

    /* pipes: [0]=read end, [1]=write end */
    int p_ib[2], p_bi[2], p_db[2], p_bd[2];          /* I<->B, D<->B */
    int p_ob[2] = {-1,-1}, p_tb[2] = {-1,-1}, p_bt[2] = {-1,-1};
    int p_nb[2] = {-1,-1}, p_bn[2] = {-1,-1};
    if (pipe(p_ib) || pipe(p_bi) || pipe(p_db) || pipe(p_bd)) { perror("pipe"); return 1; }
    if (local  && (pipe(p_ob) || pipe(p_tb) || pipe(p_bt)))   { perror("pipe"); return 1; }
    if (!local && (pipe(p_nb) || pipe(p_bn)))                 { perror("pipe"); return 1; }

    int allfds[18]; int nall = 0;
    int *plist[9] = { p_ib, p_bi, p_db, p_bd, p_ob, p_tb, p_bt, p_nb, p_bn };
    for (int i = 0; i < 9; i++)
        if (plist[i][0] != -1) { allfds[nall++] = plist[i][0]; allfds[nall++] = plist[i][1]; }

    /* ---- watchdog first (standalone only), so its pid is known ---- */
    pid_t wd = 0;
    if (local) {
        char *wargs[] = { "watchdog", itoa_dup(P.wd_timeout), NULL };
        wd = spawn("bin/watchdog", wargs, allfds, nall, NULL, 0);
    }
    char *wds = itoa_dup((int)wd);
    char *ms  = itoa_dup(mode);

    /* ---- blackboard ---- */
    {
        int keep[8] = { p_ib[0], p_bi[1], p_db[0], p_bd[1], -1, -1, -1, -1 };
        int nk = 4;
        if (local) { keep[nk++] = p_ob[0]; keep[nk++] = p_tb[0]; keep[nk++] = p_bt[1]; }
        else       { keep[nk++] = p_nb[0]; keep[nk++] = p_bn[1]; }
        char *args[] = { "blackboard", ms,
            itoa_dup(p_ib[0]), itoa_dup(p_bi[1]),
            itoa_dup(p_db[0]), itoa_dup(p_bd[1]),
            itoa_dup(local ? p_ob[0] : -1),
            itoa_dup(local ? p_tb[0] : -1), itoa_dup(local ? p_bt[1] : -1),
            itoa_dup(local ? -1 : p_nb[0]), itoa_dup(local ? -1 : p_bn[1]),
            wds, NULL };
        spawn("bin/blackboard", args, allfds, nall, keep, nk);
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
        char *args[] = { "input", ms, itoa_dup(p_ib[1]), itoa_dup(p_bi[0]), wds, NULL };
        pid_I = spawn("bin/input", args, allfds, nall, keep, 2);
    }
    if (local) {
        /* ---- obstacle generator ---- */
        int keepO[1] = { p_ob[1] };
        char *argsO[] = { "obstacles", itoa_dup(p_ob[1]), wds, NULL };
        spawn("bin/obstacles", argsO, allfds, nall, keepO, 1);
        /* ---- target generator ---- */
        int keepT[2] = { p_tb[1], p_bt[0] };
        char *argsT[] = { "targets", itoa_dup(p_tb[1]), itoa_dup(p_bt[0]), wds, NULL };
        spawn("bin/targets", argsT, allfds, nall, keepT, 2);
    } else {
        /* ---- network process ---- */
        int keepN[2] = { p_bn[0], p_nb[1] };
        char *argsN[] = { "network", itoa_dup(mode == MODE_SERVER ? 1 : 2),
                          addr, itoa_dup(P.net_port),
                          itoa_dup(p_bn[0]), itoa_dup(p_nb[1]), NULL };
        spawn("bin/network", argsN, allfds, nall, keepN, 2);
    }

    /* master holds no pipe end: EOF must propagate between processes */
    for (int i = 0; i < nall; i++) close(allfds[i]);

    /* publish the pid list for the watchdog */
    FILE *pf = fopen(PID_FILE, "w");
    if (pf) {
        const char *names_local[] = { "watchdog", "blackboard", "drone",
                                      "input", "obstacles", "targets" };
        const char *names_net[]   = { "blackboard", "drone", "input", "network" };
        const char **names = local ? names_local : names_net;
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
        if (done == pid_I) break;          /* UI gone: shut down */
    }
    usleep(300000);                        /* grace period */
    for (int i = 0; i < nchildren; i++)
        if (children[i] > 0) kill(children[i], SIGTERM);
    while (waitpid(-1, NULL, 0) > 0) ;
    printf("simulation ended - per-process logs are in ./logs/\n");
    return 0;
}
