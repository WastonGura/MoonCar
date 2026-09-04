#include "mooncar_core.h"
#include <math.h>
#include <string.h>

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 255; p[1] = v >> 8; }
static float clamp(float v) { return fmaxf(-1.0f, fminf(1.0f, v)); }

uint16_t mc_crc(const uint8_t *data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

void mc_init(mc_core *c, float radius, float track, float kp, float ki, float kd) {
  memset(c, 0, sizeof(*c));
  c->radius = radius;
  c->track = track;
  for (int i = 0; i < 4; ++i) c->pid[i] = (mc_pid){kp, ki, kd, 0, 0};
}

void mc_receive(mc_core *c, uint8_t byte, uint32_t now_ms) {
  c->rx[c->used++] = byte;
  while (c->used >= 8) {
    if (c->rx[0] != 0xa5 || c->rx[1] != 0x5a || c->rx[2] != 1 ||
        c->rx[3] != 1 || u16(c->rx+6) != 5) goto skip;
    if (c->used < 15) return;
    if (mc_crc(c->rx, 13) != u16(c->rx+13)) goto skip;
    uint16_t seq = u16(c->rx+4);
    int16_t linear = (int16_t)u16(c->rx+8), angular = (int16_t)u16(c->rx+10);
    uint8_t flags = c->rx[12];
    uint16_t delta = (uint16_t)(seq-c->seq);
    if ((flags & ~3) || linear < -350 || linear > 350 || angular < -800 || angular > 800)
      goto skip;
    c->estop |= (flags & 2) != 0;
    if (!c->received && (flags & 1)) goto skip;
    /* A disarm packet establishes a new host session; motion requires newer sequence. */
    if (!(flags & 1) || (delta > 0 && delta < 32768)) {
      bool session_ready = c->received;
      c->seq = seq;
      c->last_command_ms = now_ms;
      c->received = true;
      c->armed = session_ready && (flags & 1) && !c->estop;
      c->linear = c->armed ? linear*0.001f : 0;
      c->angular = c->armed ? angular*0.001f : 0;
    }
    memmove(c->rx, c->rx+15, c->used-15);
    c->used -= 15;
    continue;
skip:
    memmove(c->rx, c->rx+1, --c->used);
  }
}

void mc_step(mc_core *c, uint32_t now_ms, float dt, const float velocity[4],
             bool external_stop, float pwm[4]) {
  c->estop |= external_stop;
  bool safe = c->received && (uint32_t)(now_ms-c->last_command_ms) <= 250 &&
      c->armed && !c->estop && isfinite(dt) && dt > 0 && dt <= 0.1f &&
      isfinite(c->radius) && c->radius > 0 && isfinite(c->track) && c->track > 0;
  for (int i = 0; i < 4; ++i)
    safe = safe && isfinite(velocity[i]) && isfinite(c->pid[i].kp) &&
        isfinite(c->pid[i].ki) && isfinite(c->pid[i].kd);
  if (!safe) c->armed = false;
  for (int i = 0; i < 4; ++i) {
    mc_pid *p = &c->pid[i];
    if (!safe) { p->integral = p->previous = pwm[i] = 0; continue; }
    float target = (c->linear + (i % 2 ? 1 : -1)*c->angular*c->track*0.5f)/c->radius;
    float error = target-velocity[i];
    float integral = clamp(p->integral + error*dt*p->ki);
    float output = p->kp*error + integral + p->kd*(error-p->previous)/dt;
    /* Conditional integration prevents windup against the PWM saturation limit. */
    if (fabsf(output) <= 1.0f || output*error < 0) p->integral = integral;
    pwm[i] = clamp(output);
    p->previous = error;
  }
}

size_t mc_telemetry(const mc_core *c, const int32_t ticks[4], uint16_t battery_mv,
                    uint32_t now_ms, uint8_t out[30]) {
  out[0] = 0xa5; out[1] = 0x5a; out[2] = 1; out[3] = 2;
  put16(out+4, c->seq); put16(out+6, 20);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) out[8+4*i+j] = ((uint32_t)ticks[i] >> (8*j)) & 255;
  put16(out+24, battery_mv);
  uint16_t faults = (c->estop ? 1 : 0) |
      ((!c->received || (uint32_t)(now_ms-c->last_command_ms) > 250) ? 2 : 0);
  put16(out+26, faults); put16(out+28, mc_crc(out, 28));
  return 30;
}
