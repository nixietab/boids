# Boids Screensaver

A screensaver written in C that simulates **boids** (flocking behavior) and has various mathematical patterns i find neat

- Includes several pattern modes:
  - Lissajous
  - Rose
  - Hypocycloid
  - Butterfly
  - Maurer Rose
  - Spirograph
  - Fermat Spiral
  - Cardioid
- Automatically cycles through modes in screensaver mode.
- Keyboard controls for switching modes manually.

## Requirements

- X11 (for Linux-based systems)
- C compiler (e.g., `gcc`)

## Build and Run

```sh
gcc -o boids boids.c -lX11 -lm
./boids
