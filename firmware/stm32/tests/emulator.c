#define _POSIX_C_SOURCE 200809L
#include "mooncar_core.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static uint32_t millis(void) {
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint32_t)((uint64_t)t.tv_sec*1000+t.tv_nsec/1000000);
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  int fd = atoi(argv[1]);
  fcntl(fd, F_SETFL, O_NONBLOCK);
  mc_core c; mc_init(&c, 0.16f, 0.59f, 0.12f, 0.2f, 0);
  float speed[4] = {0}, pwm[4], positions[4] = {0};
  uint8_t buffer[128], packet[30]; int32_t ticks[4];
  struct timespec delay = {0, 10000000};
  while (true) {
    ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return 0;
    uint32_t now = millis();
    for (ssize_t i = 0; i < count; ++i) mc_receive(&c, buffer[i], now);
    mc_step(&c, now, 0.01f, speed, false, pwm);
    for (int i = 0; i < 4; ++i) {
      speed[i] += (pwm[i]*8-speed[i])*0.2f;
      positions[i] += speed[i]*0.01f;
      ticks[i] = (int32_t)(positions[i]*1000/(2*3.14159265f));
    }
    mc_telemetry(&c, ticks, 12000, now, packet);
    if (write(fd, packet, 30) < 0 && errno != EAGAIN) return 0;
    nanosleep(&delay, NULL);
  }
}
