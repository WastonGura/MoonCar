#include "mooncar_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void command(mc_core *c, uint16_t seq, uint8_t flags, uint32_t now, bool corrupt) {
  uint8_t data[15] = {0xa5, 0x5a, 1, 1, seq & 255, seq >> 8, 5, 0, 200, 0, 0, 0, flags, 0, 0};
  uint16_t crc = mc_crc(data, 13);
  data[13] = crc & 255; data[14] = crc >> 8;
  if (corrupt) data[9] ^= 1;
  for (int i = 0; i < 15; ++i) mc_receive(c, data[i], now);
}
int main(void) {
  mc_core c; float speed[4] = {0}, pwm[4];
  mc_init(&c, 0.16f, 0.59f, 0.1f, 0.1f, 0.0f);
  assert(mc_crc((const uint8_t *)"123456789", 9) == 0x29b1);
  mc_step(&c, 0, 0.01f, speed, false, pwm); assert(pwm[0] == 0);
  command(&c, 1, 0, 1, false);
  command(&c, 2, 1, 2, false);
  mc_step(&c, 3, 0.01f, speed, false, pwm); assert(pwm[0] > 0 && pwm[0] <= 1);
  command(&c, 3, 1, 240, true); assert(c.seq == 2);
  command(&c, 2, 1, 240, false); assert(c.last_command_ms == 2);
  mc_step(&c, 253, 0.01f, speed, false, pwm); assert(pwm[0] == 0 && !c.armed);
  command(&c, 4, 1, 254, false);
  mc_step(&c, 255, 0.01f, speed, true, pwm); assert(pwm[0] == 0 && c.estop);
  command(&c, 5, 1, 256, false); assert(!c.armed);
  for (int i = 0; i < 1000; ++i) mc_receive(&c, (uint8_t)i, 257);
  assert(c.used < sizeof(c.rx));
  mc_init(&c, 0.16f, 0.59f, 0.1f, 0.1f, 0.0f);
  command(&c, 65535, 0, UINT32_MAX-10, false);
  command(&c, 0, 1, UINT32_MAX-5, false);
  mc_step(&c, 4, 0.01f, speed, false, pwm); assert(pwm[0] > 0);
  speed[0] = NAN;
  mc_step(&c, 5, 0.01f, speed, false, pwm); assert(pwm[0] == 0 && pwm[1] == 0);
  puts("firmware core: CRC, framing, sequence, watchdog, estop, rollover, NaN passed");
}
