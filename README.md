# Drone Simulator Project - Assignment 1+2+3
**Author:** Richard Albert King Mechoda  
**Student:** 8525970  
**Course:** Advanced and Robot Programming (86736), MSc Robotics Engineering, University of Genoa  

**Group:** solo submission (I enrolled in the MSc in January 2026, after the groups were formed; the situation has been communicated to the professor)  

**Assignment 3 testing:** both modalities (server and client) were tested peer to peer between two independent instances over the local network, since no companion group was available.  


# Project Overview

The Drone Simulator is a real-time, multi-process system that models the navigation of a drone inside a 2D operation area populated with targets and obstacles. The system uses a Blackboard Architecture: independent processes cooperate by reading and writing the shared state of the world through a central Blackboard Server, connected with unnamed pipes and multiplexed with select(). The drone is a point mass with inertia and viscous friction, moved by forces given from the keyboard, while obstacles (and the borders of the area, which act as geo-fences) repel it with the Latombe/Khatib potential field model. A Watchdog process monitors the health of the system through signals and stops everything if some process is not computing anymore. In assignment 3 the simulator gains a Network Capability: two instances connect over a TCP socket, one as Server and one as Client (each instance can take both roles), exchanging positions with a simple string protocol where every message is acknowledged.

## System Architecture

This diagram shows the processes and the IPC data flow. The Master acts as the bootstrapper, solid arrows are pipes, dotted arrows are signals.

![Architecture Diagram](assignment2/assets/architecture.png)

## Simulation Demo

A snapshot of the simulation in action (assignment 2): the blue cross is the drone, the green numbers are the targets to reach in order, the yellow dots are the obstacles. On the right there is the inspection window with the state and the score.

![Simulation Screenshot](assignment2/assets/simulation.png)

The watchdog in action: here the drone process was frozen on purpose with `kill -STOP`, and after 10 seconds of silence the watchdog sends the notification and stops the whole system.

![Watchdog Screenshot](assignment2/assets/watchdog.png)

## 1. Sketch of the Architecture

The system is a concurrent, multi-process **Blackboard Architecture**. The `Blackboard` process is the central repository: all the other processes talk only with it, using **unnamed pipes** read in a single **select()** loop. This keeps the components decoupled: each one can be replaced or stopped without touching the others. The **Watchdog** stays outside the pipe network and works only with signals.

**Data flow:**
1. **Input:** the ncurses window captures the keystrokes and sends them to the Blackboard, which forwards them to the Dynamics process as force steps.
2. **Environment:** `Obstacles` and `Targets` generate random coordinates and send them to the Blackboard.
3. **Physics:** `Dynamics` integrates the motion equation at a fixed rate and sends the new drone state to the Blackboard.
4. **Output:** the Blackboard checks reached targets and collisions, computes the score, and sends the world snapshot back to the window, which renders it.
5. **Monitoring:** every process sends a SIGUSR1 heartbeat to the `Watchdog` at each cycle of its main loop.

## 2. Active Components Definition

### A. Master (`src/master.c`)
* **Role:** system orchestrator.
* **Function:** creates the 7 pipes, forks all the processes (watchdog first, so everybody knows its pid), writes the pid registry `logs/pids.txt`, and guarantees a safe termination when the window closes.
* **Primitives:** `pipe()`, `fork()`, `execv()`, `waitpid()`, `kill()`, `sigprocmask()`.

### B. Blackboard Server (`src/blackboard.c`)
* **Role:** central hub.
* **Function:** routes the data between all the agents with non-blocking multiplexing; detects the targets reached in sequence and the collisions; computes the score; re-reads `params.txt` while the system runs.
* **Primitives:** `select()`, `read()`, `write()`, `stat()`.
* **Logic:** fd-set scanning + event driven routing. The score formula is `100*targets - 1.0*time - 0.2*distance - 25*hits`.

### C. Drone Dynamics (`src/drone.c`)
* **Role:** physics engine.
* **Function:** keeps the drone state (the last two positions per axis) and integrates the motion at a fixed period T.
* **Algorithms:**
  1. **Motion integration:** Euler method (section 3).
  2. **Repulsion model:** Latombe/Khatib potential fields, applied to the obstacles and to the four borders of the area.
  3. The repulsion is saturated near zero distance to avoid numerical explosions.

### D. Keyboard Manager + Windows (`src/input.c`)
* **Role:** input handler and renderer.
* **Function:** reads the keys without blocking (50 ms timeout), maps them to force steps, draws the playfield and the lateral inspection window.
* **Primitives:** `ncurses` (colors, non blocking getch), `select()` to drain the world updates.

### E. Targets and Obstacles (`src/targets.c`, `src/obstacles.c`)
* **Role:** random environment generators.
* **Function:** obstacles appear in random positions and disappear after a random lifetime (10 to 40 s); targets arrive in numbered batches and a new batch is generated when all are reached.
* **Logic:** random generation with a minimum distance from the drone start area.

### F. Watchdog (`src/watchdog.c`)
* **Role:** system monitor, **signal based**.
* **Function:** every process sends SIGUSR1 at each cycle; the handler is installed with `sigaction(SA_SIGINFO)`, so `si_pid` tells which process sent the beat. If one process stays silent more than `wd_timeout` seconds the watchdog writes a notification in the log and stops the whole system.
* Note: the master blocks SIGUSR1 before forking, and the watchdog unblocks it after installing the handler. Without this, an early heartbeat could kill the watchdog before it is ready (the default action of SIGUSR1 is termination).

### G. Network (`src/network.c`, assignment 3 only)
* **Role:** socket endpoint, both roles.
* **Function:** implements the exchange protocol of section 4. In networked mode the generators and the watchdog are turned off, as required.
* **Primitives:** `socket()`, `bind()`, `listen()`, `accept()`, `connect()`, line oriented `read()`/`write()`.

## 3. Digital Solution of the Dynamic Equation

The drone has two degrees of freedom, a mass M and a viscous coefficient K. The vector equation is split into the two scalar components; for the X component:

$$(2) \quad \sum F_x = M \frac{d^2x}{dt^2} + K \frac{dx}{dt}$$

Applying Euler's method with integration interval $T$:

$$(3) \quad \sum F_{x,i} = M \left( \frac{x_i - 2x_{i-1} + x_{i-2}}{T^2} \right) + K \left( \frac{x_i - x_{i-1}}{T} \right)$$

which solved for the new position gives the update used in the code:

$$x_i = \frac{F T^2 + M (2x_{i-1} - x_{i-2}) + K T x_{i-1}}{M + K T}$$

* T: integration interval, 10 to 100 ms (50 ms default, set in `params.txt`).
* Suggested values M = 1 kg, K = 1 N s/m, area 100 m. With a constant force the drone reaches the terminal velocity F/K, which is an easy way to check the integration is correct.
* Repulsion (obstacles and walls): F = eta (1/d - 1/rho) / d^2 for d < rho, directed away from the obstacle. Beyond rho the obstacle is not perceived; near d = 0 the force is clamped.

## 4. Network Protocol (Assignment 3)

Two assignment-2 applications connect through a TCP socket (port 5500). All the data travel as character strings and every message has an acknowledgement, with sequential loops (no select on the socket), exactly as the protocol notes require:

```
server:  snd ok;        rcv ook
         snd size l h;  rcv sok
         loop [ quit -> snd q; rcv qok; exit
                snd drone; snd x y;  rcv dok
                snd obst;  rcv x y;  snd pok ]

client:  rcv ok;        snd ook
         rcv size l h;  snd sok
         loop [ rcv x
                x == q     -> snd qok; exit
                x == drone -> rcv x y; snd dok
                x == obst  -> snd x y; rcv pok ]
```

The coordinates are exchanged in a **virtual coordinate system** with the origin in the bottom-left corner of the server window, in window characters (the window size travels in the `size` message), so two groups with different internal conventions can still interoperate. The server sends its drone and receives the client drone as its **single obstacle** (the generators are off); the client reproduces the server window size, shows the server drone as a red X and moves its own drone as usual.

Server instance, with the client drone received from the socket shown as the obstacle:

![Server Screenshot](assignment3/assets/server.png)

Client instance, same moment, opposite point of view:

![Client Screenshot](assignment3/assets/client.png)

## 5. Directories

```
/ARP-Assignments/
|
|-- README.md                this file
|
|-- assignment1/             B, D, I + parameter file
|   |-- Makefile
|   |-- params.txt
|   |-- README.md
|   |-- assets/              diagram + screenshot
|   `-- src/                 master, blackboard, drone, input, common
|
|-- assignment2/             full system: B, D, I, O, T, W + logfiles
|   |-- Makefile
|   |-- params.txt
|   |-- README.md
|   |-- assets/
|   `-- src/                 + obstacles, targets, watchdog
|
`-- assignment3/             standalone / server / client over TCP
    |-- Makefile
    |-- params.txt
    |-- README.md
    |-- assets/
    `-- src/                 + network
```

`bin/` (the executables) and `logs/` (pids.txt and the per-process logs) are created by `make` and at runtime, they are not part of the repository.

## 6. Operational Instructions

### Controls
```
w / e / r : Up-Left    / Up     / Up-Right
s / d / f : Left       / BRAKE  / Right
x / c / v : Down-Left  / Down   / Down-Right

p         : suspend / resume
b         : reset
q         : quit
```
Every key press adds one force step (1 N) in that direction; the opposite key decreases it; `d` puts the command force to zero.

### Visual Legend
```
+    -> drone (blue)
1-9  -> targets (green, to reach in order)
o    -> obstacles (yellow, repulsive)
X    -> drone of the other instance (red, assignment 3)
```

### Real-Time Parameters

All the parameters are in `params.txt` (mass, viscosity, force step, integration period, rho, eta, number of obstacles and targets, watchdog timeout, network address and port). The file is re-read while the simulator runs: edit it, save, and the new values are applied immediately.

## 7. Installation and Running

### Prerequisites
```
sudo apt-get install build-essential libncurses-dev
```

### Compilation
From the assignment folder:
```
make
```

### Execution
```
./bin/master
```

### Running modes (assignment 3)
```
./bin/master                  asks: local or networked, then server or client
./bin/master standalone       same as assignment 2
./bin/master server           waits for a client on the port of params.txt
./bin/master client <addr>    connects to the server (find its address with: ip addr)
```

Stop the simulation with `q` in the window. Quitting the server also closes the client, through the q / qok messages of the protocol.
