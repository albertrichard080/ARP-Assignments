#include "common.h"

void params_default(params_t *p)
{
    memset(p, 0, sizeof *p);
    p->mass          = 1.0;
    p->visc          = 1.0;
    p->force_step    = 1.0;
    p->max_cmd_force = 20.0;
    p->rho           = 5.0;
    p->eta           = 1.0;
    p->max_rep_force = 15.0;
    p->sim_period_ms = 50;          /* 20 Hz */
    p->n_obst        = 8;
    p->n_targ        = 7;
    p->obst_min_life = 10;
    p->obst_max_life = 40;
    p->target_radius    = 2.0;
    p->collision_radius = 1.0;
    p->wd_timeout    = 10;
    p->net_port      = 5500;
    strcpy(p->net_addr, "127.0.0.1");
    p->win_w         = 72;
    p->win_h         = 22;
}

int params_load(const char *path, params_t *p)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256], key[64], val[128];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        if (sscanf(line, " %63[^= ] = %127s", key, val) != 2) continue;
        if      (!strcmp(key, "mass"))             p->mass = atof(val);
        else if (!strcmp(key, "viscosity"))        p->visc = atof(val);
        else if (!strcmp(key, "force_step"))       p->force_step = atof(val);
        else if (!strcmp(key, "max_cmd_force"))    p->max_cmd_force = atof(val);
        else if (!strcmp(key, "rho"))              p->rho = atof(val);
        else if (!strcmp(key, "eta"))              p->eta = atof(val);
        else if (!strcmp(key, "max_rep_force"))    p->max_rep_force = atof(val);
        else if (!strcmp(key, "sim_period_ms"))    p->sim_period_ms = atoi(val);
        else if (!strcmp(key, "n_obstacles"))      p->n_obst = atoi(val);
        else if (!strcmp(key, "n_targets"))        p->n_targ = atoi(val);
        else if (!strcmp(key, "obst_min_life"))    p->obst_min_life = atoi(val);
        else if (!strcmp(key, "obst_max_life"))    p->obst_max_life = atoi(val);
        else if (!strcmp(key, "target_radius"))    p->target_radius = atof(val);
        else if (!strcmp(key, "collision_radius")) p->collision_radius = atof(val);
        else if (!strcmp(key, "wd_timeout"))       p->wd_timeout = atoi(val);
        else if (!strcmp(key, "net_port"))         p->net_port = atoi(val);
        else if (!strcmp(key, "net_addr"))         { strncpy(p->net_addr, val, 63); p->net_addr[63] = 0; }
        else if (!strcmp(key, "win_w"))            p->win_w = atoi(val);
        else if (!strcmp(key, "win_h"))            p->win_h = atoi(val);
    }
    fclose(f);
    if (p->n_obst > MAX_OBST) p->n_obst = MAX_OBST;
    if (p->n_targ > MAX_TARG) p->n_targ = MAX_TARG;
    if (p->sim_period_ms < 10)  p->sim_period_ms = 10;
    if (p->sim_period_ms > 100) p->sim_period_ms = 100;
    return 0;
}

int msg_write(int fd, const msg_t *m)
{
    ssize_t n = write(fd, m, sizeof *m);
    if (n == sizeof *m) return 1;
    if (n < 0 && (errno == EPIPE || errno == EBADF)) return 0;
    return -1;
}

int msg_read(int fd, msg_t *m)
{
    char *p = (char *)m;
    size_t got = 0;
    while (got < sizeof *m) {
        ssize_t n = read(fd, p + got, sizeof *m - got);
        if (n == 0) return 0;
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += (size_t)n;
    }
    return 1;
}

FILE *log_open(const char *procname)
{
    char path[256];
    mkdir(LOG_DIR, 0777);
    snprintf(path, sizeof path, LOG_DIR "/%s.log", procname);
    FILE *f = fopen(path, "w");
    if (f) setvbuf(f, NULL, _IOLBF, 0);
    return f;
}

void log_line(FILE *lf, const char *fmt, ...)
{
    if (!lf) return;
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    fprintf(lf, "[%02d:%02d:%02d] ", tm.tm_hour, tm.tm_min, tm.tm_sec);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(lf, fmt, ap);
    va_end(ap);
    fputc('\n', lf);
}

double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void heartbeat(pid_t wd_pid)
{
    if (wd_pid > 0) kill(wd_pid, SIGUSR1);
}
