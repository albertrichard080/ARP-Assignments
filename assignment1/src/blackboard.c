/*
 * blackboard.c - process B: blackboard server (assignment 1)
 *
 * Holds the geometrical state of the world (drone position, velocity,
 * forces) and mediates the exchanges between the keyboard manager and
 * the drone process with a single select() loop over the pipes
 * (process / pipes / select model).
 *
 * It also polls the parameter file: when params.txt changes on disk the
 * new values are applied immediately and forwarded to the drone process.
 *
 * argv: fd_from_I fd_to_I fd_from_D fd_to_D wd_pid
 */
#include "common.h"

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "blackboard: bad args\n"); return 1; }
    int fd_iI = atoi(argv[1]), fd_oI = atoi(argv[2]);
    int fd_iD = atoi(argv[3]), fd_oD = atoi(argv[4]);
    pid_t wd  = (pid_t)atoi(argv[5]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("blackboard");
    log_line(lf, "blackboard started, pid %d", getpid());

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);
    struct stat st;
    time_t param_mtime = (stat(PARAM_FILE, &st) == 0) ? st.st_mtime : 0;
    double next_param_check = now_sec() + 1.0;

    world_t W; memset(&W, 0, sizeof W);
    W.drone.x = WORLD_W / 2; W.drone.y = WORLD_H / 2;

    double t_start = now_sec(), pause_started = 0, paused_total = 0;
    vec2 prev_pos = W.drone;
    int have_prev = 0;

    for (;;) {
        heartbeat(wd);

        fd_set rf; FD_ZERO(&rf);
        int maxfd = -1;
        int fds[2] = { fd_iI, fd_iD };
        for (int i = 0; i < 2; i++)
            if (fds[i] >= 0) { FD_SET(fds[i], &rf); if (fds[i] > maxfd) maxfd = fds[i]; }

        struct timeval tv = { 0, 100000 };       /* 100 ms */
        int r = select(maxfd + 1, &rf, NULL, NULL, &tv);
        if (r < 0) { if (errno == EINTR) continue; break; }

        /* ---------- keyboard manager ---------- */
        if (fd_iI >= 0 && FD_ISSET(fd_iI, &rf)) {
            msg_t m;
            if (msg_read(fd_iI, &m) <= 0) break;
            if (m.type == MSG_QUIT) {
                log_line(lf, "quit requested by user");
                msg_t q = { .type = MSG_QUIT };
                if (fd_oD >= 0) msg_write(fd_oD, &q);
                break;
            }
            if (m.type == MSG_KEY) {
                if (m.u.key == 'p') {              /* suspend / resume */
                    W.paused = !W.paused;
                    if (W.paused) pause_started = now_sec();
                    else paused_total += now_sec() - pause_started;
                    log_line(lf, "%s", W.paused ? "paused" : "resumed");
                }
                if (m.u.key == 'b') {              /* reset */
                    W.dist = 0;
                    t_start = now_sec(); paused_total = 0; have_prev = 0;
                    log_line(lf, "reset");
                }
                if (fd_oD >= 0) msg_write(fd_oD, &m);   /* forward to D */
            }
        }

        /* ---------- drone dynamics ---------- */
        if (fd_iD >= 0 && FD_ISSET(fd_iD, &rf)) {
            msg_t m;
            if (msg_read(fd_iD, &m) <= 0) break;
            if (m.type == MSG_POS) {
                W.drone = m.u.state.pos;
                W.vel   = m.u.state.vel;
                W.cmd_f = m.u.state.cmd_f;
                W.tot_f = m.u.state.tot_f;
                if (have_prev) W.dist += hypot(W.drone.x - prev_pos.x,
                                               W.drone.y - prev_pos.y);
                prev_pos = W.drone; have_prev = 1;
                if (!W.paused)
                    W.elapsed = now_sec() - t_start - paused_total;

                /* refresh the window */
                if (fd_oI >= 0) {
                    msg_t w = { .type = MSG_WORLD };
                    w.u.world = W;
                    if (msg_write(fd_oI, &w) <= 0) break;
                }
            }
        }

        /* ---------- parameter file polling (real-time tuning) ---------- */
        if (now_sec() > next_param_check) {
            next_param_check = now_sec() + 1.0;
            if (stat(PARAM_FILE, &st) == 0 && st.st_mtime != param_mtime) {
                param_mtime = st.st_mtime;
                params_load(PARAM_FILE, &P);
                log_line(lf, "parameter file reloaded");
                if (fd_oD >= 0) {
                    msg_t pm = { .type = MSG_PARAMS };
                    pm.u.params = P;
                    msg_write(fd_oD, &pm);
                }
            }
        }
    }
    log_line(lf, "blackboard exiting");
    fclose(lf);
    return 0;
}
