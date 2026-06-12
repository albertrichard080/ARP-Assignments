/*
 * blackboard.c - process B: blackboard server
 *
 * Holds the geometrical state of the world (drone, obstacles, targets,
 * score) and mediates every exchange between the other processes with a
 * single select() loop over the pipes (process / pipes / select model).
 *
 * It also polls the parameter file: when params.txt changes on disk the
 * new values are applied immediately and forwarded to the drone process.
 *
 * argv: mode fd_from_I fd_to_I fd_from_D fd_to_D fd_from_O fd_from_T
 *       fd_to_T fd_from_N fd_to_N wd_pid
 * mode: 0 standalone, 1 server, 2 client   (unused fds are -1)
 */
#include "common.h"

#define MODE_STANDALONE 0
#define MODE_SERVER     1
#define MODE_CLIENT     2

int main(int argc, char **argv)
{
    if (argc < 12) { fprintf(stderr, "blackboard: bad args\n"); return 1; }
    int mode    = atoi(argv[1]);
    int fd_iI   = atoi(argv[2]),  fd_oI = atoi(argv[3]);
    int fd_iD   = atoi(argv[4]),  fd_oD = atoi(argv[5]);
    int fd_iO   = atoi(argv[6]);
    int fd_iT   = atoi(argv[7]),  fd_oT = atoi(argv[8]);
    int fd_iN   = atoi(argv[9]),  fd_oN = atoi(argv[10]);
    pid_t wd    = (pid_t)atoi(argv[11]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("blackboard");
    log_line(lf, "blackboard started, pid %d, mode %d", getpid(), mode);

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);
    struct stat st;
    time_t param_mtime = (stat(PARAM_FILE, &st) == 0) ? st.st_mtime : 0;
    double next_param_check = now_sec() + 1.0;

    world_t W; memset(&W, 0, sizeof W);
    W.drone.x = WORLD_W / 2; W.drone.y = WORLD_H / 2;
    if (mode == MODE_SERVER) { W.win_w = P.win_w; W.win_h = P.win_h; }

    double t_start = now_sec(), pause_started = 0, paused_total = 0;
    vec2 prev_pos = W.drone;
    int have_prev = 0;
    int targets_requested = 1;   /* T sends the first batch unprompted */
    double next_score_log = now_sec() + 2.0;

    for (;;) {
        heartbeat(wd);

        fd_set rf; FD_ZERO(&rf);
        int maxfd = -1;
        int fds[4] = { fd_iI, fd_iD, fd_iO, fd_iT };
        for (int i = 0; i < 4; i++)
            if (fds[i] >= 0) { FD_SET(fds[i], &rf); if (fds[i] > maxfd) maxfd = fds[i]; }
        if (fd_iN >= 0) { FD_SET(fd_iN, &rf); if (fd_iN > maxfd) maxfd = fd_iN; }

        struct timeval tv = { 0, 100000 };       /* 100 ms */
        int r = select(maxfd + 1, &rf, NULL, NULL, &tv);
        if (r < 0) { if (errno == EINTR) continue; break; }

        /* ---------- keyboard manager ---------- */
        if (fd_iI >= 0 && FD_ISSET(fd_iI, &rf)) {
            msg_t m;
            if (msg_read(fd_iI, &m) <= 0) break;
            if (m.type == MSG_QUIT) {
                log_line(lf, "quit requested by user, final score %.1f", W.score);
                msg_t q = { .type = MSG_QUIT };
                if (fd_oD >= 0) msg_write(fd_oD, &q);
                if (fd_oN >= 0) msg_write(fd_oN, &q);
                break;
            }
            if (m.type == MSG_KEY) {
                if (m.u.key == 'p') {              /* suspend / resume */
                    W.paused = !W.paused;
                    if (W.paused) pause_started = now_sec();
                    else paused_total += now_sec() - pause_started;
                    log_line(lf, "%s", W.paused ? "paused" : "resumed");
                }
                if (m.u.key == 'b') {              /* reset mission */
                    W.score = 0; W.reached = 0; W.hits = 0; W.dist = 0;
                    t_start = now_sec(); paused_total = 0; have_prev = 0;
                    log_line(lf, "mission reset");
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

                /* targets are reached IN SEQUENCE: only the lowest id counts */
                if (W.targ.n > 0) {
                    int lo = 0;
                    for (int i = 1; i < W.targ.n; i++)
                        if (W.targ.id[i] < W.targ.id[lo]) lo = i;
                    double d = hypot(W.drone.x - W.targ.pos[lo].x,
                                     W.drone.y - W.targ.pos[lo].y);
                    if (d < P.target_radius) {
                        log_line(lf, "target %d reached", W.targ.id[lo]);
                        W.reached++;
                        W.targ.pos[lo] = W.targ.pos[W.targ.n - 1];
                        W.targ.id[lo]  = W.targ.id[W.targ.n - 1];
                        W.targ.n--;
                    }
                }
                /* collision penalty (counted once per second at most) */
                static double last_hit = 0;
                double tnow = now_sec();
                for (int i = 0; i < W.obst.n; i++) {
                    double d = hypot(W.drone.x - W.obst.pos[i].x,
                                     W.drone.y - W.obst.pos[i].y);
                    if (d < P.collision_radius && tnow - last_hit > 1.0) {
                        W.hits++; last_hit = tnow;
                        log_line(lf, "obstacle hit #%d", W.hits);
                    }
                }
                if (W.have_netdrone) {  /* peer drone repulses too (server) */
                    /* nothing to do here: it is already in W.obst */
                }

                if (!W.paused)
                    W.elapsed = now_sec() - t_start - paused_total;

                /* overall score: weighted formula */
                W.score = 100.0 * W.reached
                        -   1.0 * W.elapsed
                        -   0.2 * W.dist
                        -  25.0 * W.hits;

                /* notify the network process of our position */
                if (fd_oN >= 0) {
                    msg_t d2 = { .type = MSG_DRONEPOS };
                    d2.u.netpos = W.drone;
                    msg_write(fd_oN, &d2);
                }
                /* refresh the windows */
                if (fd_oI >= 0) {
                    msg_t w = { .type = MSG_WORLD };
                    w.u.world = W;
                    if (msg_write(fd_oI, &w) <= 0) break;
                }
            }
        }

        /* ---------- obstacle generator ---------- */
        if (fd_iO >= 0 && FD_ISSET(fd_iO, &rf)) {
            msg_t m;
            if (msg_read(fd_iO, &m) <= 0) { fd_iO = -1; }
            else if (m.type == MSG_OBST) {
                W.obst = m.u.obst;
                if (fd_oD >= 0) msg_write(fd_oD, &m);   /* drone needs them */
            }
        }

        /* ---------- target generator ---------- */
        if (fd_iT >= 0 && FD_ISSET(fd_iT, &rf)) {
            msg_t m;
            if (msg_read(fd_iT, &m) <= 0) { fd_iT = -1; }
            else if (m.type == MSG_TARG) {
                W.targ = m.u.targ;
                targets_requested = 0;
                log_line(lf, "new batch of %d targets", W.targ.n);
            }
        }
        if (fd_oT >= 0 && W.targ.n == 0 && !targets_requested) {
            msg_t q = { .type = MSG_TREQ };
            msg_write(fd_oT, &q);
            targets_requested = 1;
        }

        /* ---------- network process (assignment 3) ---------- */
        if (fd_iN >= 0 && FD_ISSET(fd_iN, &rf)) {
            msg_t m;
            if (msg_read(fd_iN, &m) <= 0) { fd_iN = -1; }
            else switch (m.type) {
            case MSG_OBST:        /* server: the client drone is our obstacle */
                W.obst = m.u.obst;
                W.net_connected = 1;
                if (fd_oD >= 0) msg_write(fd_oD, &m);
                break;
            case MSG_NETDRONE:    /* client: position of the server drone */
                W.netdrone = m.u.netpos;
                W.have_netdrone = 1;
                W.net_connected = 1;
                break;
            case MSG_SIZE:        /* client: window size imposed by server */
                W.win_w = m.u.size.w; W.win_h = m.u.size.h;
                log_line(lf, "window size from server: %dx%d", W.win_w, W.win_h);
                break;
            case MSG_QUIT:        /* peer closed: shut everything down */
                log_line(lf, "connection closed by peer, shutting down");
                { msg_t q = { .type = MSG_QUIT };
                  if (fd_oD >= 0) msg_write(fd_oD, &q);
                  if (fd_oI >= 0) msg_write(fd_oI, &q); }
                fclose(lf);
                return 0;
            default: break;
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
            if (now_sec() > next_score_log) {
                next_score_log = now_sec() + 5.0;
                log_line(lf, "score %.1f | targets %d | hits %d | dist %.1fm | t %.0fs",
                         W.score, W.reached, W.hits, W.dist, W.elapsed);
            }
        }
    }
    log_line(lf, "blackboard exiting");
    fclose(lf);
    return 0;
}
