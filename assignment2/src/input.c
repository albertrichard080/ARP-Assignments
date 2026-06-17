/*
 * input.c - process I: keyboard manager + ncurses windows (assignment 2)
 *
 * Full screen playfield (the geofence) plus a small lateral inspection
 * window showing position, velocity, forces and the score in real time.
 *
 * Motion keys (cluster suggested by the assignment):
 *      w e r        up-left    up    up-right
 *      s d f        left      BRAKE  right
 *      x c v        down-left  down  down-right
 * Other keys:  p = suspend/resume, b = reset, q = quit.
 *
 * argv: fd_to_B fd_from_B wd_pid
 */
#include "common.h"
#include <ncurses.h>

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "input: bad args\n"); return 1; }
    int fd_out = atoi(argv[1]);
    int fd_in  = atoi(argv[2]);
    pid_t wd   = (pid_t)atoi(argv[3]);

    signal(SIGPIPE, SIG_IGN);
    FILE *lf = log_open("input");
    log_line(lf, "input/window started, pid %d", getpid());

    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    timeout(50);                     /* getch() returns every 50 ms */
    start_color(); use_default_colors();
    init_pair(1, COLOR_BLUE,    -1);   /* drone           */
    init_pair(2, COLOR_GREEN,   -1);   /* targets         */
    init_pair(3, COLOR_YELLOW,  -1);   /* obstacles (orange) */
    init_pair(4, COLOR_CYAN,    -1);   /* frame / labels  */

    world_t W; memset(&W, 0, sizeof W);
    W.drone.x = WORLD_W / 2; W.drone.y = WORLD_H / 2;
    int have_world = 0;

    for (;;) {
        heartbeat(wd);

        int ch = getch();
        /* the sheet allows arrows too: map them onto the key cluster */
        if (ch == KEY_UP)    ch = 'e';
        else if (ch == KEY_DOWN)  ch = 'c';
        else if (ch == KEY_LEFT)  ch = 's';
        else if (ch == KEY_RIGHT) ch = 'f';
        if (ch != ERR) {
            if (ch == 'q') {
                msg_t m = { .type = MSG_QUIT };
                msg_write(fd_out, &m);
                log_line(lf, "quit key");
                break;
            }
            if (strchr("wersdfxcvpb", ch)) {
                msg_t m = { .type = MSG_KEY, .u.key = ch };
                if (msg_write(fd_out, &m) <= 0) break;
                log_line(lf, "key '%c'", ch);
            }
        }

        /* drain everything B sent, keep the latest world snapshot */
        for (;;) {
            fd_set rf; FD_ZERO(&rf); FD_SET(fd_in, &rf);
            struct timeval tv = { 0, 0 };
            if (select(fd_in + 1, &rf, NULL, NULL, &tv) <= 0) break;
            msg_t m;
            int n = msg_read(fd_in, &m);
            if (n <= 0) goto done;                /* B is gone */
            if (m.type == MSG_WORLD) { W = m.u.world; have_world = 1; }
            if (m.type == MSG_QUIT)  goto done;
        }

        /* ---------------- drawing ---------------- */
        erase();
        int rows, cols; getmaxyx(stdscr, rows, cols);
        int insp_w = 32;
        int pw = cols - insp_w - 1;               /* playfield width  */
        int ph = rows;                            /* playfield height */
        if (pw < 20) pw = 20;
        if (ph < 10) ph = 10;

        attron(COLOR_PAIR(4));
        for (int x = 0; x < pw; x++) { mvaddch(0, x, '-'); mvaddch(ph - 1, x, '-'); }
        for (int y = 0; y < ph; y++) { mvaddch(y, 0, '|'); mvaddch(y, pw - 1, '|'); }
        mvaddch(0, 0, '+'); mvaddch(0, pw - 1, '+');
        mvaddch(ph - 1, 0, '+'); mvaddch(ph - 1, pw - 1, '+');
        mvprintw(0, 2, "[ geofence %dx%d ]", pw, ph);
        attroff(COLOR_PAIR(4));

        /* world (origin bottom-left) -> screen cells inside the border:
         * x: 0..WORLD_W -> 1..pw-2,  y: 0..WORLD_H -> ph-2..1 (y flipped) */
        double sx = (pw - 3) / WORLD_W, sy = (ph - 3) / WORLD_H;
        #define SCRX(wx) (1 + (int)((wx) * sx + 0.5))
        #define SCRY(wy) (ph - 2 - (int)((wy) * sy + 0.5))

        attron(COLOR_PAIR(3) | A_BOLD);
        for (int i = 0; i < W.obst.n; i++)
            mvaddch(SCRY(W.obst.pos[i].y), SCRX(W.obst.pos[i].x), 'o');
        attroff(COLOR_PAIR(3) | A_BOLD);

        /* the next target to reach is the lowest id: highlight it */
        int nx = -1;
        for (int i = 0; i < W.targ.n; i++)
            if (nx < 0 || W.targ.id[i] < W.targ.id[nx]) nx = i;
        attron(COLOR_PAIR(2) | A_BOLD);
        for (int i = 0; i < W.targ.n; i++) {
            if (i == nx) attron(A_REVERSE);
            mvaddch(SCRY(W.targ.pos[i].y), SCRX(W.targ.pos[i].x), '0' + W.targ.id[i]);
            if (i == nx) attroff(A_REVERSE);
        }
        attroff(COLOR_PAIR(2) | A_BOLD);

        attron(COLOR_PAIR(1) | A_BOLD);           /* our drone: blue cross */
        mvaddch(SCRY(W.drone.y), SCRX(W.drone.x), '+');
        attroff(COLOR_PAIR(1) | A_BOLD);

        /* ------------- inspection window ------------- */
        int ix = cols - insp_w;
        attron(COLOR_PAIR(4));
        for (int y = 0; y < rows; y++) mvaddch(y, ix - 1, '|');
        attroff(COLOR_PAIR(4));
        attron(A_BOLD);
        mvprintw( 1, ix, "DRONE SIMULATOR");
        attroff(A_BOLD);
        mvprintw( 3, ix, "pos   : %7.2f %7.2f m", W.drone.x, W.drone.y);
        mvprintw( 4, ix, "vel   : %7.2f %7.2f m/s", W.vel.x, W.vel.y);
        mvprintw( 5, ix, "cmd F : %7.2f %7.2f N", W.cmd_f.x, W.cmd_f.y);
        mvprintw( 6, ix, "tot F : %7.2f %7.2f N", W.tot_f.x, W.tot_f.y);
        mvprintw( 8, ix, "targets reached : %d", W.reached);
        if (W.targ.n > 0)
            mvprintw( 9, ix, "next target     : %d  (%d left)", W.targ.id[nx], W.targ.n);
        else
            mvprintw( 9, ix, "targets left    : 0");
        mvprintw(10, ix, "obstacles       : %d", W.obst.n);
        mvprintw(11, ix, "hits            : %d", W.hits);
        mvprintw(12, ix, "distance        : %.1f m", W.dist);
        mvprintw(13, ix, "time            : %.0f s", W.elapsed);
        attron(A_BOLD);
        mvprintw(15, ix, "SCORE : %.1f", W.score);
        attroff(A_BOLD);
        mvprintw(17, ix, "status: %s", W.paused ? "PAUSED" : "RUNNING");
        mvprintw(19, ix, "keys  w e r   p pause");
        mvprintw(20, ix, "      s d f   b reset");
        mvprintw(21, ix, "      x c v   q quit");
        mvprintw(22, ix, "      (arrows work too)");
        if (!have_world) mvprintw(rows / 2, 2, " waiting for blackboard ... ");
        refresh();
    }
done:
    endwin();
    log_line(lf, "input exiting");
    fclose(lf);
    return 0;
}
