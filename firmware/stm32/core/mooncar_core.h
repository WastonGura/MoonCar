#ifndef MOONCAR_CORE_H
#define MOONCAR_CORE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  float kp, ki, kd, integral, previous;
} mc_pid;
typedef struct {
  uint8_t rx[74];
  size_t used;
  uint16_t seq;
  uint32_t last_command_ms;
  bool received, armed, estop;
  float linear, angular, radius, track;
  mc_pid pid[4];
} mc_core;

void mc_init(mc_core *c, float radius, float track, float kp, float ki, float kd);
void mc_receive(mc_core *c, uint8_t byte, uint32_t now_ms);
void mc_step(mc_core *c, uint32_t now_ms, float dt, const float velocity[4],
             bool external_stop, float pwm[4]);
size_t mc_telemetry(const mc_core *c, const int32_t ticks[4], uint16_t battery_mv,
                    uint32_t now_ms, uint8_t out[30]);
uint16_t mc_crc(const uint8_t *data, size_t size);
#endif
