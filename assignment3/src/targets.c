/*
 * targets.c - process T: target generator
 *
 * Sends a batch of numbered targets (to be reached in sequence) to the
 * blackboard; generates a fresh batch whenever the blackboard asks for
 * one (all targets reached).  argv: fd_to_B fd_from_B wd_pid
 */
#include "common.h"

static double frand(double lo, double hi)
{
    return lo + (hi - lo) * (rand() / (double)RAND_MAX);
}

static int send_batch(int fd, int n, FILE *lf)
{
    msg_t m; memset(&m, 0, sizeof m);
    m.type = MSG_TARG;
    m.u.targ.n = n;
    for (int i = 0; i < n; i++) {
        m.u.targ.id[i] = i + 1;
        m.u.targ.pos[i].x = frand(5, WORLD_W - 5);
        m.u.targ.pos[i].y = frand(5, WORLD_H - 5);
    }
    log_line(lf, "generated batch of %d targets", n);
    return msg_write(fd, &m);
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "targets: bad args\n"); return 1; }
    int fd_out = atoi(argv[1]);
    int fd_in  = atoi(argv[2]);
    pid_t wd   = (pid_t)atoi(argv[3]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("targets");
    log_line(lf, "target generator started, pid %d", getpid());
    srand((unsigned)(getpid() * 2246822519u));

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);
    if (send_batch(fd_out, P.n_targ, lf) <= 0) return 0;

    for (;;) {
        heartbeat(wd);
        fd_set rf; FD_ZERO(&rf); FD_SET(fd_in, &rf);
        struct timeval tv = { 1, 0 };
        int r = select(fd_in + 1, &rf, NULL, NULL, &tv);
        if (r < 0 && errno != EINTR) break;
        if (r > 0 && FD_ISSET(fd_in, &rf)) {
            msg_t m;
            int n = msg_read(fd_in, &m);
            if (n <= 0) break;                   /* B closed: exit */
            if (m.type == MSG_TREQ) {
                params_load(PARAM_FILE, &P);
                if (send_batch(fd_out, P.n_targ, lf) <= 0) break;
            }
            if (m.type == MSG_QUIT) break;
        }
    }
    log_line(lf, "target generator exiting");
    fclose(lf);
    return 0;
}
