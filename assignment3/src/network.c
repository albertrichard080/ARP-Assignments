/*
 * network.c - process N: socket communication (assignment 3)
 *
 * Implements the given protocol (all data as character strings, every
 * message acknowledged, sequential loop - no select on the socket):
 *
 *   server:  snd ok;  rcv ook
 *            snd size l h;  rcv sok
 *            loop [ if quit -> snd q; rcv qok; exit
 *                   snd drone; snd x y; rcv dok
 *                   snd obst;  rcv x y; snd pok ]
 *
 *   client:  rcv ok;  snd ook
 *            rcv size l h;  snd sok
 *            loop [ rcv x
 *                   x == q     -> snd qok; exit
 *                   x == drone -> rcv x y; snd dok
 *                   x == obst  -> snd x y; rcv pok ]
 *
 * Coordinates are exchanged in the VIRTUAL coordinate system: origin at
 * the bottom-left corner of the server window, units = window characters
 * (the window size travels in the "size" message).  alfa = 0 for us.
 *
 * argv: mode(1=server,2=client) addr port fd_from_B fd_to_B
 */
#include "common.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ctype.h>

static FILE *lf;
static int sock = -1;

/* --- line-oriented string exchange ------------------------------- */
static int snd_line(const char *s)
{
    char buf[128];
    int n = snprintf(buf, sizeof buf, "%s\n", s);
    log_line(lf, "snd: %s", s);
    return (write(sock, buf, n) == n) ? 1 : -1;
}

static int rcv_line(char *buf, int len)
{
    int i = 0;
    while (i < len - 1) {
        char c;
        ssize_t n = read(sock, &c, 1);
        if (n == 0) return 0;                  /* connection closed */
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        if (c == '\n') break;
        if (c != '\r') buf[i++] = c;
    }
    buf[i] = 0;
    log_line(lf, "rcv: %s", buf);
    return 1;
}

/* --- world <-> virtual coordinate conversion ---------------------- */
/* virtual system: origin bottom-left, units = characters of the       */
/* server playfield (w x h).  Our world is already bottom-left, alfa=0 */
static int VW, VH;
static void world2virt(vec2 w, double *vx, double *vy)
{
    *vx = w.x / WORLD_W * (VW - 1);
    *vy = w.y / WORLD_H * (VH - 1);
}
static vec2 virt2world(double vx, double vy)
{
    vec2 w;
    w.x = vx / (VW - 1) * WORLD_W;
    w.y = vy / (VH - 1) * WORLD_H;
    if (w.x < 0) w.x = 0;
    if (w.x > WORLD_W) w.x = WORLD_W;
    if (w.y < 0) w.y = 0;
    if (w.y > WORLD_H) w.y = WORLD_H;
    return w;
}

/* latest drone position fed by B; drain the pipe, keep the newest */
static vec2 mypos = { WORLD_W / 2, WORLD_H / 2 };
static int quit_requested = 0;
static void drain_from_B(int fd)
{
    for (;;) {
        fd_set rf; FD_ZERO(&rf); FD_SET(fd, &rf);
        struct timeval tv = { 0, 0 };
        if (select(fd + 1, &rf, NULL, NULL, &tv) <= 0) return;
        msg_t m;
        if (msg_read(fd, &m) <= 0) { quit_requested = 1; return; }
        if (m.type == MSG_DRONEPOS) mypos = m.u.netpos;
        if (m.type == MSG_QUIT)     quit_requested = 1;
    }
}

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "network: bad args\n"); return 1; }
    int is_server = (atoi(argv[1]) == 1);
    const char *addr = argv[2];
    int port  = atoi(argv[3]);
    int fd_in = atoi(argv[4]);
    int fd_out= atoi(argv[5]);

    signal(SIGPIPE, SIG_IGN);
    lf = log_open("network");
    log_line(lf, "network started, pid %d, %s, %s:%d",
             getpid(), is_server ? "SERVER" : "CLIENT", addr, port);

    params_t P; params_default(&P); params_load(PARAM_FILE, &P);
    VW = P.win_w; VH = P.win_h;

    char buf[128];

    if (is_server) {
        /* ---------------- SERVER ---------------- */
        int lsock = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        struct sockaddr_in sa = {0};
        sa.sin_family = AF_INET;
        sa.sin_addr.s_addr = INADDR_ANY;
        sa.sin_port = htons(port);
        if (bind(lsock, (struct sockaddr *)&sa, sizeof sa) < 0 ||
            listen(lsock, 1) < 0) {
            log_line(lf, "bind/listen failed: %s", strerror(errno));
            return 1;
        }
        log_line(lf, "waiting for a client on port %d ...", port);
        sock = accept(lsock, NULL, NULL);
        if (sock < 0) { log_line(lf, "accept failed"); return 1; }
        close(lsock);
        log_line(lf, "client connected");

        /* handshake */
        if (snd_line("ok") < 0 || rcv_line(buf, sizeof buf) <= 0 ||
            strcmp(buf, "ook")) { log_line(lf, "handshake failed"); goto out; }
        snprintf(buf, sizeof buf, "size %d %d", VW, VH);
        if (snd_line(buf) < 0 || rcv_line(buf, sizeof buf) <= 0 ||
            strcmp(buf, "sok")) { log_line(lf, "size not acked"); goto out; }

        /* sequential exchange loop, ~20 Hz */
        for (;;) {
            drain_from_B(fd_in);
            if (quit_requested) {
                snd_line("q");
                rcv_line(buf, sizeof buf);          /* qok */
                log_line(lf, "closed connection");
                break;
            }
            /* our drone -> client */
            double vx, vy;
            world2virt(mypos, &vx, &vy);
            snd_line("drone");
            snprintf(buf, sizeof buf, "%.2f %.2f", vx, vy);
            if (snd_line(buf) < 0) break;
            if (rcv_line(buf, sizeof buf) <= 0 || strcmp(buf, "dok")) break;

            /* client drone -> our single obstacle */
            snd_line("obst");
            if (rcv_line(buf, sizeof buf) <= 0) break;
            if (sscanf(buf, "%lf %lf", &vx, &vy) == 2) {
                snd_line("pok");
                msg_t m; memset(&m, 0, sizeof m);
                m.type = MSG_OBST;
                m.u.obst.n = 1;
                m.u.obst.pos[0] = virt2world(vx, vy);
                m.u.obst.die_at[0] = 0;             /* never expires */
                if (msg_write(fd_out, &m) <= 0) break;
            }
            usleep(50000);
        }
    } else {
        /* ---------------- CLIENT ---------------- */
        sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in sa = {0};
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        inet_pton(AF_INET, addr, &sa.sin_addr);
        log_line(lf, "connecting to %s:%d ...", addr, port);
        /* retry while the server comes up */
        int tries = 0;
        while (connect(sock, (struct sockaddr *)&sa, sizeof sa) < 0) {
            if (++tries > 60) { log_line(lf, "connect failed"); return 1; }
            sleep(1);
        }
        log_line(lf, "connected");

        /* handshake */
        if (rcv_line(buf, sizeof buf) <= 0 || strcmp(buf, "ok")) goto out;
        snd_line("ook");
        if (rcv_line(buf, sizeof buf) <= 0) goto out;
        /* tolerant parsing: some groups send "size l h", others "l,h" */
        for (char *c = buf; *c; c++) if (*c == ',' || *c == ';') *c = ' ';
        char *digits = buf;
        while (*digits && !isdigit((unsigned char)*digits)) digits++;
        if (sscanf(digits, "%d %d", &VW, &VH) == 2) {
            snd_line("sok");
            msg_t m; memset(&m, 0, sizeof m);     /* reproduce the window */
            m.type = MSG_SIZE;
            m.u.size.w = VW; m.u.size.h = VH;
            msg_write(fd_out, &m);
        }

        for (;;) {
            if (rcv_line(buf, sizeof buf) <= 0) break;
            if (!strcmp(buf, "q")) {              /* server is leaving */
                snd_line("qok");
                log_line(lf, "server closed the session");
                msg_t m = { .type = MSG_QUIT };
                msg_write(fd_out, &m);
                break;
            }
            if (!strcmp(buf, "drone")) {          /* server drone position */
                double vx, vy;
                if (rcv_line(buf, sizeof buf) <= 0) break;
                for (char *c = buf; *c; c++) if (*c == ',') *c = ' ';
                if (sscanf(buf, "%lf %lf", &vx, &vy) == 2) {
                    snd_line("dok");
                    msg_t m; memset(&m, 0, sizeof m);
                    m.type = MSG_NETDRONE;
                    m.u.netpos = virt2world(vx, vy);
                    if (msg_write(fd_out, &m) <= 0) break;
                }
            } else if (!strcmp(buf, "obst")) {    /* send our drone as obstacle */
                drain_from_B(fd_in);
                if (quit_requested) break;        /* our UI quit: just drop */
                double vx, vy;
                world2virt(mypos, &vx, &vy);
                snprintf(buf, sizeof buf, "%.2f %.2f", vx, vy);
                if (snd_line(buf) < 0) break;
                if (rcv_line(buf, sizeof buf) <= 0) break;   /* pok */
            }
        }
    }
out:
    if (sock >= 0) close(sock);
    log_line(lf, "network exiting");
    fclose(lf);
    return 0;
}
