/*
 * drone.c - process D: drone dynamics
 *
 * Solves   sum(F) = M p'' + K p'   with the Euler discretisation
 *
 *   x_i = ( F T^2 + M (2 x_{i-1} - x_{i-2}) + K T x_{i-1} ) / ( M + K T )
 *
 * Forces:
 *   1. command forces   - keyboard, 8 directions, |F| steps of force_step
 *   2. repulsion forces - obstacles, Latombe/Khatib model
 *   3. repulsion forces - the four geofence walls (same model)
 *
 * argv: fd_from_B fd_to_B wd_pid
 */
#include "common.h"

static const double SQ2 = 0.70710678118654752440;

/* 8 command directions: r, l, u, d, ur, ul, dr, dl */
static void key_to_dir(int key, double *dx, double *dy, int *brake)
{
    *dx = *dy = 0; *brake = 0;
    switch (key) {
    case 'f': *dx =  1; break;              /* right       */
    case 's': *dx = -1; break;              /* left        */
    case 'e': *dy =  1; break;              /* up          */
    case 'c': *dy = -1; break;              /* down        */
    case 'r': *dx =  SQ2; *dy =  SQ2; break;/* up-right    */
    case 'w': *dx = -SQ2; *dy =  SQ2; break;/* up-left     */
    case 'v': *dx =  SQ2; *dy = -SQ2; break;/* down-right  */
    case 'x': *dx = -SQ2; *dy = -SQ2; break;/* down-left   */
    case 'd': *brake = 1; break;            /* centre key: brake */
    }
}

/* Khatib/Latombe repulsion from a point at distance d (d < rho):
 *   F = eta (1/d - 1/rho) / d^2 , directed away from the obstacle  */
static double rep_mag(double d, const params_t *P)
{
    if (d >= P->rho) return 0;
    if (d < 0.05) d = 0.05;
    double f = P->eta * (1.0 / d - 1.0 / P->rho) / (d * d);
    if (f > P->max_rep_force) f = P->max_rep_force;
    return f;
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "drone: bad args\n"); return 1; }
    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);
    pid_t wd   = (pid_t)atoi(argv[3]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("drone");
    log_line(lf, "drone started, pid %d", getpid());

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);

    /* state: previous two positions per axis (Euler memory) */
    double x1 = WORLD_W / 2, x2 = WORLD_W / 2;   /* x_{i-1}, x_{i-2} */
    double y1 = WORLD_H / 2, y2 = WORLD_H / 2;
    vec2 cmd_f = {0, 0};
    obstacles_t obst; memset(&obst, 0, sizeof obst);
    int paused = 0;

    double next_tick = now_sec() + P.sim_period_ms * 1e-3;

    for (;;) {
        heartbeat(wd);

        /* wait for messages from B until the next integration instant */
        double dt = next_tick - now_sec();
        if (dt < 0) dt = 0;
        struct timeval tv = { (time_t)dt, (suseconds_t)((dt - (time_t)dt) * 1e6) };
        fd_set rf; FD_ZERO(&rf); FD_SET(fd_in, &rf);
        int r = select(fd_in + 1, &rf, NULL, NULL, &tv);
        if (r < 0 && errno != EINTR) break;

        if (r > 0 && FD_ISSET(fd_in, &rf)) {
            msg_t m;
            int n = msg_read(fd_in, &m);
            if (n <= 0) break;                       /* B closed: exit */
            switch (m.type) {
            case MSG_KEY: {
                double dx, dy; int brake;
                key_to_dir(m.u.key, &dx, &dy, &brake);
                if (brake) { cmd_f.x = cmd_f.y = 0; }
                else {
                    cmd_f.x += dx * P.force_step;
                    cmd_f.y += dy * P.force_step;
                    double m2 = hypot(cmd_f.x, cmd_f.y);
                    if (m2 > P.max_cmd_force) {      /* saturate */
                        cmd_f.x *= P.max_cmd_force / m2;
                        cmd_f.y *= P.max_cmd_force / m2;
                    }
                }
                if (m.u.key == 'b') {                /* reset */
                    x1 = x2 = WORLD_W / 2; y1 = y2 = WORLD_H / 2;
                    cmd_f.x = cmd_f.y = 0;
                    log_line(lf, "reset");
                }
                if (m.u.key == 'p') paused = !paused;
                break;
            }
            case MSG_OBST:   obst = m.u.obst; break;
            case MSG_PARAMS: P = m.u.params;
                             log_line(lf, "parameters reloaded (M=%.2f K=%.2f T=%dms)",
                                      P.mass, P.visc, P.sim_period_ms);
                             break;
            case MSG_QUIT:   log_line(lf, "quit"); fclose(lf); return 0;
            default: break;
            }
            continue;       /* keep draining until the tick expires */
        }

        /* ---- integration tick ---- */
        next_tick += P.sim_period_ms * 1e-3;
        if (paused) {
            x2 = x1; y2 = y1;          /* freeze: kill velocity memory */
        } else {
            double T = P.sim_period_ms * 1e-3;
            double Fx = cmd_f.x, Fy = cmd_f.y;

            /* repulsion: obstacles + geofence walls (Latombe/Khatib) */
            double tnow = now_sec();
            double Rx = 0, Ry = 0;
            for (int i = 0; i < obst.n; i++) {
                if (obst.die_at[i] > 0 && tnow > obst.die_at[i]) continue;
                double ddx = x1 - obst.pos[i].x, ddy = y1 - obst.pos[i].y;
                double d = hypot(ddx, ddy);
                double f = rep_mag(d, &P);
                if (f > 0 && d > 1e-9) { Rx += f * ddx / d; Ry += f * ddy / d; }
            }
            Rx += rep_mag(x1, &P);                    /* left wall   */
            Rx -= rep_mag(WORLD_W - x1, &P);          /* right wall  */
            Ry += rep_mag(y1, &P);                    /* bottom wall */
            Ry -= rep_mag(WORLD_H - y1, &P);          /* top wall    */

            /* turn the repulsion into a "virtual key pressure": project it
             * on the 8 command directions and keep the strongest one, as
             * suggested by the assignment sheet */
            if (Rx != 0 || Ry != 0) {
                static const double DX[8] = {1,-1,0,0, SQ2, SQ2,-SQ2,-SQ2};
                static const double DY[8] = {0,0,1,-1, SQ2,-SQ2, SQ2,-SQ2};
                double best = 0; int bi = -1;
                for (int k = 0; k < 8; k++) {
                    double dot = Rx * DX[k] + Ry * DY[k];
                    if (dot > best) { best = dot; bi = k; }
                }
                if (bi >= 0) { Fx += best * DX[bi]; Fy += best * DY[bi]; }
            }

            double den = P.mass + P.visc * T;
            double xi = (Fx * T * T + P.mass * (2 * x1 - x2) + P.visc * T * x1) / den;
            double yi = (Fy * T * T + P.mass * (2 * y1 - y2) + P.visc * T * y1) / den;

            /* hard clamp inside the geofence (safety net) */
            if (xi < 0) { xi = 0; x1 = xi; }
            if (xi > WORLD_W) { xi = WORLD_W; x1 = xi; }
            if (yi < 0) { yi = 0; y1 = yi; }
            if (yi > WORLD_H) { yi = WORLD_H; y1 = yi; }

            x2 = x1; x1 = xi;
            y2 = y1; y1 = yi;

            msg_t out; memset(&out, 0, sizeof out);
            out.type = MSG_POS;
            out.u.state.pos.x = xi; out.u.state.pos.y = yi;
            out.u.state.vel.x = (x1 - x2) / T;
            out.u.state.vel.y = (y1 - y2) / T;
            out.u.state.cmd_f = cmd_f;
            out.u.state.tot_f.x = Fx; out.u.state.tot_f.y = Fy;
            if (msg_write(fd_out, &out) <= 0) break;
        }
    }
    log_line(lf, "drone exiting");
    fclose(lf);
    return 0;
}
