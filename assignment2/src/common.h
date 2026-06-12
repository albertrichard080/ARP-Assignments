/*
 * common.h - shared definitions for the ARP drone simulator
 *
 * Processes: master, B (blackboard server), D (drone dynamics),
 *            I (keyboard manager + ncurses windows), O (obstacle generator),
 *            T (target generator), W (watchdog), N (network, assignment 3).
 *
 * All inter-process communication uses POSIX unnamed pipes carrying
 * fixed-size msg_t structures (size < PIPE_BUF, hence writes are atomic),
 * multiplexed with select().  The watchdog is signal based (SIGUSR1).
 */
#ifndef ARP_COMMON_H
#define ARP_COMMON_H

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdarg.h>

/* ------------------------------------------------------------------ */
/* world geometry: operation area (geofence) in meters,                */
/* origin at the BOTTOM-LEFT corner (matches the assignment-3 virtual  */
/* coordinate system, alfa = 0)                                        */
/* ------------------------------------------------------------------ */
#define WORLD_W 100.0
#define WORLD_H 100.0

#define MAX_OBST 30
#define MAX_TARG 9

#define PARAM_FILE "params.txt"
#define LOG_DIR    "logs"
#define PID_FILE   "logs/pids.txt"

/* run-time parameters: all live in params.txt and are re-read while
 * the simulator runs (the blackboard polls the file mtime) */
typedef struct {
    double mass;          /* M  [kg]               */
    double visc;          /* K  [N*s/m]            */
    double force_step;    /* |F| increment per key press [N] */
    double max_cmd_force; /* saturation of the command force */
    double rho;           /* obstacle perception radius [m]  */
    double eta;           /* repulsion gain                  */
    double max_rep_force; /* clamp for the repulsive force   */
    int    sim_period_ms; /* integration interval T [ms]     */
    int    n_obst;        /* obstacles kept alive            */
    int    n_targ;        /* targets per batch               */
    int    obst_min_life; /* obstacle lifetime range [s]     */
    int    obst_max_life;
    double target_radius; /* distance at which a target is "reached" [m] */
    double collision_radius; /* distance counting as obstacle hit [m]    */
    int    wd_timeout;    /* watchdog inactivity timeout [s] */
    int    net_port;      /* assignment 3: TCP port          */
    char   net_addr[64];  /* assignment 3: server address    */
    int    win_w, win_h;  /* playfield size in characters (sent to client) */
} params_t;

typedef struct { double x, y; } vec2;

typedef struct {
    int  n;
    vec2 pos[MAX_OBST];
    double die_at[MAX_OBST];   /* absolute time the obstacle disappears */
} obstacles_t;

typedef struct {
    int  n;
    vec2 pos[MAX_TARG];
    int  id[MAX_TARG];         /* targets must be reached in this order */
} targets_t;

/* message types travelling on the pipes */
typedef enum {
    MSG_KEY = 1,    /* I -> B -> D : one keyboard character             */
    MSG_POS,        /* D -> B      : drone state after one Euler step   */
    MSG_OBST,       /* O -> B -> D : full obstacle list (also N -> B)   */
    MSG_TARG,       /* T -> B      : new batch of targets               */
    MSG_TREQ,       /* B -> T      : please generate a new batch        */
    MSG_WORLD,      /* B -> I      : snapshot for drawing               */
    MSG_PARAMS,     /* B -> D      : parameters (re)loaded              */
    MSG_NETDRONE,   /* N -> B      : position of the remote drone       */
    MSG_DRONEPOS,   /* B -> N      : our drone position (for the peer)  */
    MSG_SIZE,       /* N -> B      : window size imposed by the server  */
    MSG_QUIT        /* shutdown request                                 */
} mtype_t;

/* world snapshot used by the ncurses window */
typedef struct {
    vec2 drone, vel, cmd_f, tot_f;
    obstacles_t obst;
    targets_t   targ;
    double score;
    int    reached;        /* targets reached so far     */
    int    hits;           /* obstacle collisions        */
    double dist;           /* distance travelled [m]     */
    double elapsed;        /* simulated mission time [s] */
    int    paused;
    vec2   netdrone;       /* assignment 3: peer drone   */
    int    have_netdrone;
    int    win_w, win_h;   /* >0: playfield size imposed by the network */
    int    net_connected;
} world_t;

typedef struct {
    mtype_t type;
    union {
        int key;
        struct { vec2 pos, vel, cmd_f, tot_f; } state;
        obstacles_t obst;
        targets_t   targ;
        world_t     world;
        params_t    params;
        vec2        netpos;
        struct { int w, h; } size;
    } u;
} msg_t;

/* ------------------------------------------------------------------ */
/* helpers (common.c)                                                  */
/* ------------------------------------------------------------------ */
void  params_default(params_t *p);
int   params_load(const char *path, params_t *p);

int   msg_write(int fd, const msg_t *m);   /* 1 ok, 0 closed, -1 error */
int   msg_read (int fd, msg_t *m);         /* 1 ok, 0 EOF,    -1 error */

FILE *log_open(const char *procname);
void  log_line(FILE *lf, const char *fmt, ...);

double now_sec(void);
void  heartbeat(pid_t wd_pid);             /* SIGUSR1 to the watchdog  */

#endif
