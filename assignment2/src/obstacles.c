/*
 * obstacles.c - process O: obstacle generator
 *
 * Obstacles appear at random positions and disappear after a random
 * lifetime.  The full list is sent to the blackboard every time it
 * changes.  argv: fd_to_B wd_pid
 */
#include "common.h"

static double frand(double lo, double hi)
{
    return lo + (hi - lo) * (rand() / (double)RAND_MAX);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "obstacles: bad args\n"); return 1; }
    int fd_out = atoi(argv[1]);
    pid_t wd   = (pid_t)atoi(argv[2]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("obstacles");
    log_line(lf, "obstacle generator started, pid %d", getpid());
    srand((unsigned)(getpid() * 2654435761u));

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);

    obstacles_t ob; memset(&ob, 0, sizeof ob);

    for (;;) {
        heartbeat(wd);
        params_load(PARAM_FILE, &P);          /* real-time tuning */

        int changed = 0;
        double tnow = now_sec();

        /* remove expired obstacles */
        for (int i = 0; i < ob.n; ) {
            if (tnow > ob.die_at[i]) {
                ob.pos[i] = ob.pos[ob.n - 1];
                ob.die_at[i] = ob.die_at[ob.n - 1];
                ob.n--; changed = 1;
            } else i++;
        }
        /* spawn new ones, keeping clear of the drone start area */
        while (ob.n < P.n_obst && ob.n < MAX_OBST) {
            vec2 p;
            do {
                p.x = frand(3, WORLD_W - 3);
                p.y = frand(3, WORLD_H - 3);
            } while (hypot(p.x - WORLD_W/2, p.y - WORLD_H/2) < 8.0);
            ob.pos[ob.n] = p;
            ob.die_at[ob.n] = tnow + frand(P.obst_min_life, P.obst_max_life);
            ob.n++; changed = 1;
        }

        if (changed) {
            msg_t m; memset(&m, 0, sizeof m);
            m.type = MSG_OBST;
            m.u.obst = ob;
            if (msg_write(fd_out, &m) <= 0) break;   /* B closed: exit */
            log_line(lf, "%d obstacles alive", ob.n);
        }
        usleep(500000);                              /* 2 Hz check */
    }
    log_line(lf, "obstacle generator exiting");
    fclose(lf);
    return 0;
}
