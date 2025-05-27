/*
 * Boids Screensaver
 * -----------------
 * A simple boids algorithm screensaver implemented in C.
 *
 * Author: Nixietab
 * License: MIT
 * Date: 2025-05-27
 * Repository: https://github.com/nixietab/boids
 */

#include <X11/Xlib.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <X11/keysym.h>
#include <sys/time.h>
#include <stdio.h>

#define NUM_BOIDS 500
#define MAX_SPEED 4.0
#define NEIGHBOR_RADIUS 50
#define ALIGNMENT_WEIGHT 0.05
#define COHESION_WEIGHT 0.01
#define SEPARATION_WEIGHT 0.15
#define PATTERN_FORCE 0.2
#define PATTERN_SEPARATION_WEIGHT 0.001
#define PI 3.14159265358979323846
#define PHI 1.61803398875
#define E 2.71828182846

// Time intervals for auto (screensaver) mode (in seconds)
#define BOIDS_TIME 30
#define PATTERN_TIME 35

typedef struct {
    float x, y;
    float vx, vy;
    float target_x, target_y;
    int index;
} Boid;

typedef enum {
    MODE_NORMAL,
    MODE_LISSAJOUS,
    MODE_ROSE,
    MODE_HYPOCYCLOID,
    MODE_BUTTERFLY,
    MODE_MAURER_ROSE,
    MODE_SPIROGRAPH,
    MODE_FERMAT_SPIRAL,
    MODE_CARDIOID
} FlockingMode;

Boid boids[NUM_BOIDS];
FlockingMode current_mode = MODE_NORMAL;
float pattern_time = 0.0;
int auto_mode = 0;
time_t last_mode_change;
FlockingMode last_pattern_mode = MODE_LISSAJOUS;

float get_scale_factor(int width, int height) {
    return fmin(width, height) * 0.3;
}

float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void limit_speed(Boid *b) {
    float speed = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (speed > MAX_SPEED) {
        b->vx = (b->vx / speed) * MAX_SPEED;
        b->vy = (b->vy / speed) * MAX_SPEED;
    }
}

void init_boids(int width, int height) {
    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids[i].x = rand() % width;
        boids[i].y = rand() % height;
        boids[i].vx = ((rand() % 100) / 50.0f - 1.0f) * MAX_SPEED;
        boids[i].vy = ((rand() % 100) / 50.0f - 1.0f) * MAX_SPEED;
        boids[i].target_x = boids[i].x;
        boids[i].target_y = boids[i].y;
        boids[i].index = i;
    }
}

void calculate_lissajous_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float a = 3, b = 2;
    *x = width/2 + scale * sin(a * t);
    *y = height/2 + scale * sin(b * t + PI/2);
}

void calculate_rose_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float k = 3;
    float r = scale * sin(k * t);
    *x = width/2 + r * cos(t);
    *y = height/2 + r * sin(t);
}

void calculate_hypocycloid_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float R = scale;
    float r = R/4;
    float d = r;
    
    *x = width/2 + (R-r) * cos(t) + d * cos((R-r)*t/r);
    *y = height/2 + (R-r) * sin(t) - d * sin((R-r)*t/r);
}

void calculate_butterfly_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float r = exp(cos(t)) - 2 * cos(4*t) + pow(sin(t/12), 5);
    *x = width/2 + scale/2 * sin(t) * r;
    *y = height/2 + scale/2 * cos(t) * r;
}

void calculate_maurer_rose_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height) * 1.5;
    float n = 7;
    float d = 71;
    float k = t * d;
    float r = scale * (0.8 + 0.2 * sin(n * k * PI/180));
    *x = width/2 + r * cos(k * PI/180);
    *y = height/2 + r * sin(k * PI/180);
}

void calculate_spirograph_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float R = scale * 0.8;
    float r = R * 0.4;
    float d = r * 0.8;
    
    *x = width/2 + (R-r) * cos(t) + d * cos((R-r) * t/r);
    *y = height/2 + (R-r) * sin(t) - d * sin((R-r) * t/r);
}

void calculate_fermat_spiral_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float a = scale * 0.5;
    float r = a * sqrt(t);
    float angle = t * 5;
    
    *x = width/2 + r * cos(angle);
    *y = height/2 + r * sin(angle);
}

void calculate_cardioid_position(float t, float *x, float *y, int width, int height) {
    float scale = get_scale_factor(width, height);
    float a = scale * 0.9;
    float r = a * (1 + cos(t));
    
    *x = width/2 + r * cos(t);
    *y = height/2 + r * sin(t);
}

void apply_separation_force(Boid *boid, float weight) {
    float avoid_x = 0, avoid_y = 0;
    int count = 0;

    for (int j = 0; j < NUM_BOIDS; ++j) {
        if (boid->index == j) continue;
        float dx = boids[j].x - boid->x;
        float dy = boids[j].y - boid->y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist < NEIGHBOR_RADIUS/2 && dist > 0) {
            avoid_x -= dx;
            avoid_y -= dy;
            count++;
        }
    }

    if (count > 0) {
        boid->vx += avoid_x * weight;
        boid->vy += avoid_y * weight;
    }
}

void apply_pattern_force(Boid *boid, int width, int height) {
    float target_x = 0, target_y = 0;
    float t = (float)(boid->index) / NUM_BOIDS * 10.0 + pattern_time;
    // Keyboard stuff
    switch(current_mode) {
        case MODE_LISSAJOUS:
            calculate_lissajous_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_ROSE:
            calculate_rose_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_HYPOCYCLOID:
            calculate_hypocycloid_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_BUTTERFLY:
            calculate_butterfly_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_MAURER_ROSE:
            calculate_maurer_rose_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_SPIROGRAPH:
            calculate_spirograph_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_FERMAT_SPIRAL:
            calculate_fermat_spiral_position(t, &target_x, &target_y, width, height);
            break;
        case MODE_CARDIOID:
            calculate_cardioid_position(t, &target_x, &target_y, width, height);
            break;
        default:
            return;
    }

    float dx = target_x - boid->x;
    float dy = target_y - boid->y;
    float dist = sqrtf(dx * dx + dy * dy);
    
    if (dist > 0) {
        boid->vx += (dx / dist) * PATTERN_FORCE;
        boid->vy += (dy / dist) * PATTERN_FORCE;
    }
    
    apply_separation_force(boid, PATTERN_SEPARATION_WEIGHT);
}

FlockingMode get_next_pattern_mode() {
    FlockingMode next_mode;
    do {
        next_mode = (rand() % 8) + 1;
    } while (next_mode == last_pattern_mode);
    last_pattern_mode = next_mode;
    return next_mode;
}

void check_auto_mode_timing() {
    if (!auto_mode) return;
    
    time_t current_time = time(NULL);
    int elapsed = current_time - last_mode_change;
    
    if (current_mode == MODE_NORMAL && elapsed >= BOIDS_TIME) {
        current_mode = get_next_pattern_mode();
        pattern_time = 0;
        last_mode_change = current_time;
    }
    else if (current_mode != MODE_NORMAL && elapsed >= PATTERN_TIME) {
        current_mode = MODE_NORMAL;
        pattern_time = 0;
        last_mode_change = current_time;
    }
}

void update_boids(int width, int height) {
    for (int i = 0; i < NUM_BOIDS; ++i) {
        if (current_mode == MODE_NORMAL) {
            float avg_vx = 0, avg_vy = 0;
            float center_x = 0, center_y = 0;
            float avoid_x = 0, avoid_y = 0;
            int count = 0;

            for (int j = 0; j < NUM_BOIDS; ++j) {
                if (i == j) continue;
                float dx = boids[j].x - boids[i].x;
                float dy = boids[j].y - boids[i].y;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist < NEIGHBOR_RADIUS && dist > 0) {
                    avg_vx += boids[j].vx;
                    avg_vy += boids[j].vy;
                    center_x += boids[j].x;
                    center_y += boids[j].y;
                    if (dist < NEIGHBOR_RADIUS / 2) {
                        avoid_x -= dx;
                        avoid_y -= dy;
                    }
                    count++;
                }
            }

            if (count > 0) {
                avg_vx /= count;
                avg_vy /= count;
                center_x /= count;
                center_y /= count;

                boids[i].vx += (avg_vx - boids[i].vx) * ALIGNMENT_WEIGHT;
                boids[i].vy += (avg_vy - boids[i].vy) * ALIGNMENT_WEIGHT;
                boids[i].vx += (center_x - boids[i].x) * COHESION_WEIGHT;
                boids[i].vy += (center_y - boids[i].y) * COHESION_WEIGHT;
                boids[i].vx += avoid_x * SEPARATION_WEIGHT;
                boids[i].vy += avoid_y * SEPARATION_WEIGHT;
            }
        } else {
            apply_pattern_force(&boids[i], width, height);
        }

        limit_speed(&boids[i]);

        boids[i].x += boids[i].vx;
        boids[i].y += boids[i].vy;

        if (boids[i].x < 0) boids[i].x += width;
        if (boids[i].y < 0) boids[i].y += height;
        if (boids[i].x >= width) boids[i].x -= width;
        if (boids[i].y >= height) boids[i].y -= height;
    }

    if (current_mode != MODE_NORMAL) {
        pattern_time += 0.01;
    }
}

int main(int argc, char *argv[]) {
    Display *display = XOpenDisplay(NULL);
    if (!display) return 1;

    // Check for auto mode
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--auto") == 0) {
            auto_mode = 1;
            last_mode_change = time(NULL);
            break;
        }
    }

    int screen = DefaultScreen(display);
    Window root = RootWindow(display, screen);
    Window win;
    int use_root = 0;

    if (argc > 1 && strcmp(argv[1], "-root") == 0) {
        win = root;
        use_root = 1;
    } else {
        char *env = getenv("XSCREENSAVER_WINDOW");
        if (env) {
            win = (Window)strtoul(env, NULL, 0);
        } else {
            win = XCreateSimpleWindow(display, root, 0, 0, 800, 600, 0,
                                    BlackPixel(display, screen),
                                    BlackPixel(display, screen));
            XMapWindow(display, win);
        }
    }

    XSelectInput(display, win, KeyPressMask | KeyReleaseMask | StructureNotifyMask);
    XWindowAttributes attr;
    XGetWindowAttributes(display, win, &attr);
    int width = attr.width;
    int height = attr.height;

    // Create off-screen buffer
    Pixmap buffer = XCreatePixmap(display, win, width, height, 
                                 DefaultDepth(display, screen));

    GC gc = XCreateGC(display, win, 0, NULL);
    XSetForeground(display, gc, WhitePixel(display, screen));

    srand(time(NULL));
    init_boids(width, height);

    struct timespec sleep_time;
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 16666667; // 60 FPS aprox

    while (1) {
        while (XPending(display)) {
            XEvent e;
            XNextEvent(display, &e);
            
            if (e.type == ConfigureNotify) {
                width = e.xconfigure.width;
                height = e.xconfigure.height;
                
                // Recreate buffer with new size
                XFreePixmap(display, buffer);
                buffer = XCreatePixmap(display, win, width, height,
                                     DefaultDepth(display, screen));
            } else if (e.type == KeyPress && !auto_mode) {
                KeySym key = XLookupKeysym(&e.xkey, 0);
                switch(key) {
                    case XK_l:
                    case XK_L:
                        current_mode = (current_mode == MODE_LISSAJOUS) ? MODE_NORMAL : MODE_LISSAJOUS;
                        pattern_time = 0;
                        break;
                    case XK_r:
                    case XK_R:
                        current_mode = (current_mode == MODE_ROSE) ? MODE_NORMAL : MODE_ROSE;
                        pattern_time = 0;
                        break;
                    case XK_y:
                    case XK_Y:
                        current_mode = (current_mode == MODE_HYPOCYCLOID) ? MODE_NORMAL : MODE_HYPOCYCLOID;
                        pattern_time = 0;
                        break;
                    case XK_b:
                    case XK_B:
                        current_mode = (current_mode == MODE_BUTTERFLY) ? MODE_NORMAL : MODE_BUTTERFLY;
                        pattern_time = 0;
                        break;
                    case XK_m:
                    case XK_M:
                        current_mode = (current_mode == MODE_MAURER_ROSE) ? MODE_NORMAL : MODE_MAURER_ROSE;
                        pattern_time = 0;
                        break;
                    case XK_s:
                    case XK_S:
                        current_mode = (current_mode == MODE_SPIROGRAPH) ? MODE_NORMAL : MODE_SPIROGRAPH;
                        pattern_time = 0;
                        break;
                    case XK_f:
                    case XK_F:
                        current_mode = (current_mode == MODE_FERMAT_SPIRAL) ? MODE_NORMAL : MODE_FERMAT_SPIRAL;
                        pattern_time = 0;
                        break;
                    case XK_c:
                    case XK_C:
                        current_mode = (current_mode == MODE_CARDIOID) ? MODE_NORMAL : MODE_CARDIOID;
                        pattern_time = 0;
                        break;
                    case XK_n:
                    case XK_N:
                        current_mode = MODE_NORMAL;
                        pattern_time = 0;
                        break;
                }
            }
        }

        check_auto_mode_timing();
        update_boids(width, height);

        // Clear buffer
        XSetForeground(display, gc, BlackPixel(display, screen));
        XFillRectangle(display, buffer, gc, 0, 0, width, height);
        XSetForeground(display, gc, WhitePixel(display, screen));

        // Draw to buffer
        for (int i = 0; i < NUM_BOIDS; ++i) {
            int x1 = (int)boids[i].x;
            int y1 = (int)boids[i].y;
            int x2 = x1 + (int)(boids[i].vx * 4);
            int y2 = y1 + (int)(boids[i].vy * 4);
            XDrawLine(display, buffer, gc, x1, y1, x2, y2);
        }

        // Copy buffer to window
        XCopyArea(display, buffer, win, gc, 0, 0, width, height, 0, 0);
        XFlush(display);
        
        nanosleep(&sleep_time, NULL);
    }

    // Cleanup
    XFreePixmap(display, buffer);
    XFreeGC(display, gc);
    XCloseDisplay(display);

    return 0;
}
