// Example using PwmIn interface library, sford
//  - Note: uses InterruptIn, so not available on p19/p20

// mode: u = user; a = pilot angle; p = full pilot

#include "mbed.h"
#include "Servo.h"
#include "PwmIn.h"
#include <cstdlib>

Serial car(USBTX, USBRX);

PwmIn steering_in(PA_11);
PwmIn throttle_in(PB_4);
PwmIn remote_in(PB_5);
//
Servo steering_out(PA_9);
Servo throttle_out(PA_10);

float st_zero = 0.5f;
float th_zero = 0.5f;

float unbin_vals[15] = {0.0f, 0.0714f, 0.1428f, 0.2142f, 0.2857f, \
    0.3571f, 0.4285f, 0.5f, 0.5714f, 0.6428f, \
    0.7142f, 0.7857f, 0.8571f, 0.9285f, 1.0f};
float bin_vals[15] = {0.0f, 0.03571f, 0.10714f, 0.17857f, 0.25f, \
    0.32142f, 0.39285f, 0.46428f, 0.53571f, 0.60714f, \
    0.67857f, 0.75f, 0.82142f, 0.89285f, 0.96428f};

void setSteering(float steering) {
    if (steering < 0 || steering > 1) return;
    steering_out = steering;
}
void setThrottle(float throttle, float limit) {
    if (throttle < 0 || throttle > 1) return;
    throttle_out = (throttle - th_zero) * limit + th_zero;
}
float unbin(char pos) {
    if (pos <= 0) return unbin_vals[0];
    if (pos > 14) return unbin_vals[14];
    return unbin_vals[pos];
}
char bin(float value) {
    for (char pos = 1; pos <= 14; pos++)
        if (bin_vals[pos] > value) return pos-1;
    return 14;
}
int main() {
//    car.baud(230400);
    car.baud(921600);
    float last_st = 0;
    float last_th = 0;
    float limit = 1;
    char mode = 'u';
    char echo = '0';
    char st_level = 0x07;
    char th_level = 0x07;
    char bin_st = 0;
    char bin_th = 0;
    char temp = 0;
    char currentMode = 'i'; // idle
    while(1) {
        float st = steering_in.pulsewidth() * 1000.0f - 1.0f;
        if (st < -0.5f) st = st_zero;
        if (abs(last_st - st)> 0.01f) {
            if (mode == 'u') setSteering(st);
            switch(mode) {
                case 'u':
                    setSteering(st);
                    break;
                case 'p':
                    if (st < 0.3f) mode = 'a';
                    setThrottle(th_zero, limit);
                    break;
                case 'a':
                    if (st > 0.7f) mode = 'p';
                    break;
            }
            last_st = st;
            bin_st = bin(st);
        }
        float th = throttle_in.pulsewidth() * 1000.0f - 1.0f;
        if (th < -0.5f) th = th_zero;
        if (abs(last_th - th)> 0.01f) {
            if (mode != 'p') setThrottle(th, limit);
            last_th = th;
            bin_th = bin(th);
        }
        float rm = remote_in.pulsewidth() * 1000.0f - 1.0f;
        if (rm < 0.5f) rm = 0.0f;
        //bin_rm = bin(rm);
        //
        if(car.readable()) {
            switch(currentMode) {
                case 'i': // idle
                    currentMode = car.getc();
                    break;
                case 'e': // echo mode
                    echo = car.getc();
                    car.printf("echo %d\n", echo);
                    currentMode = 'i';
                    break;
                case 'm': // change mode
                    mode = car.getc();
                    if (echo != '0') car.printf("mode %c\n", mode);
                    currentMode = 'i';
                    break;
                case 'r': // read mode
                    car.putc(bin_st * 16 + bin_th);
                    currentMode = 'i';
                    break;
                case 'w': // set steering; should be followed by t
                    temp = car.getc();
                    st_level = temp >> 4;
                    th_level = temp & 0x0F;
                    if (mode != 'u') setSteering(unbin(st_level));
                    if (mode == 'p') setThrottle(unbin(th_level), limit);
                    if (echo != '0') car.printf("values %.3f%.3f\n", unbin(st_level), unbin(th_level));
                    currentMode = 'i';
                    break;
                case 'L': // set throttle limit
                    temp = car.getc();
                    //car.printf("%c, %c\n", temp, temp & 0xF0);
                    if ((temp & 0xF0) == 0x00) {
                        //car.printf("unbin %c", temp);
                        limit = unbin(temp); 
                    }
                    if (echo != '0') car.printf("limit %.3f\n", limit);
                    currentMode = 'i';
                    break;
                default:
                    currentMode = 'i';
                    break;
            }
        }
    }
}
