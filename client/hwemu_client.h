/*
 * hwemu_client: what the hosted MCB's HAL calls to reach the hub.
 *
 * A plain C API (no modm, no taproot types), so taproot's PLATFORM_HOSTED
 * HAL can call it from can.cpp, uart.cpp, the IMU drivers and digital.cpp
 * with a few lines each -- that patch is the next phase; this is the library
 * it will link.
 *
 *   CAN      the hub's Unix-socket CAN bus, one socket per bus (<runtime
 *            dir>/<bus>.sock): no root, no kernel module.
 *   UART     a port's path: the hub's PTY for it (the vision link's MCB end).
 *   SPI/GPIO pinbus, the hub's Unix socket (transport/pinbus.hpp); also PWM and
 *            ADC. SPI goes to a device by name, or to "bus:<bus>", routed
 *            to whichever device's chip select is low, as on hardware.
 *   clock    sim microseconds, from the hub's clock in shared memory; and
 *            lockstep, to run one tick at a time with the hub.
 *
 * Every function returns 0 (or a non-negative value) on success and -1 on
 * failure, with errno set where the OS set it.
 */
#ifndef HWEMU_CLIENT_H_
#define HWEMU_CLIENT_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- paths ------------------------------------------------------------ */

/* The hub's runtime directory: $HWEMU_RUNTIME_DIR, or /tmp/hwemu. */
const char* hwemu_runtime_dir(void);

/* The serial port path the MCB opens for a logical port name, into `out`:
 * $HWEMU_PORT_<NAME> (upper-cased) if set, else <runtime dir>/<name>
 * (e.g. "vision_mcb" -> /tmp/hwemu/vision_mcb). Returns the length. */
int hwemu_port_path(const char* name, char* out, size_t out_size);

/* Opens a port path raw and non-blocking; returns the fd. */
int hwemu_serial_open(const char* path);

/* ---- CAN (SocketCAN) -------------------------------------------------- */

/* Connects to the hub's CAN bus `bus` ("can1": <runtime dir>/can1.sock, or
 * an absolute socket path), non-blocking; returns the fd. Every client on a
 * bus hears every other's frames, as on a real bus. */
int hwemu_can_open(const char* bus);
int hwemu_can_send(int fd, uint32_t id, int extended, const uint8_t* data, uint8_t len);
/* Non-blocking: 1 with a frame in the outputs, 0 if none waiting, -1 error. */
int hwemu_can_recv(int fd, uint32_t* id, int* extended, uint8_t* data, uint8_t* len);
void hwemu_can_close(int fd);

/* ---- pinbus (SPI and GPIO) -------------------------------------------- */

/* Connects to the hub's pinbus (path NULL: <runtime dir>/pinbus.sock) and
 * says hello as `client_name`. Returns a handle (>= 0). */
int hwemu_pinbus_connect(const char* path, const char* client_name);
void hwemu_pinbus_close(int handle);
/* One chip-select transaction on SPI device `device` (or "bus:<bus>"): `len` bytes out of
 * `tx`, as many back into `rx`. Blocks until the hub answers. Returns 0, or
 * -1 (and rx all 0xFF) if the device is unknown or the hub is gone. */
int hwemu_spi_transfer(int handle, const char* device, const uint8_t* tx, uint8_t* rx, uint16_t len);
/* A pin the sim drives (a limit switch). 1 high, 0 low, -1 unknown/error. */
int hwemu_gpio_read(int handle, const char* pin);
/* A pin the firmware drives (a laser, a solenoid, a chip select). */
int hwemu_gpio_write(int handle, const char* pin, int level);
/* A PWM output: duty in 0..1, and its frequency. */
int hwemu_pwm_write(int handle, const char* pin, float duty, uint32_t frequency_hz);
/* An ADC input: 12-bit counts (0..4095), or -1 on error. A pin nothing in the
 * sim drives reads 0, and the hub reports it as a gap. */
int hwemu_adc_read(int handle, const char* pin);

/* ---- sim gaps --------------------------------------------------------- */

/* Reports something the firmware needs that the sim doesn't model (a stub, a
 * port with no hub behind it): printed once as "[hwemu GAP] <source>:
 * <message>" and appended once to <runtime dir>/gaps/$HWEMU_PROCESS.jsonl.
 * `key` deduplicates; reporting the same source/key again does nothing. */
void hwemu_report_gap(const char* source, const char* key, const char* message);

/* ---- clock ------------------------------------------------------------ */

/* Sim time in microseconds, from the hub's clock (shm `hwemu_clock`, or
 * $HWEMU_CLOCK_SHM); a monotonic 64-bit count that never wraps. Falls back
 * to CLOCK_MONOTONIC since the first call when no hub is running. */
uint64_t hwemu_clock_micros(void);

/* Joins the hub's lockstep (socket NULL: <runtime dir>/clock.sock) as
 * `name`; returns a handle. */
int hwemu_lockstep_join(const char* socket_path, const char* name);
/* Blocks for the next tick; fills its number and sim time (us). 0, or -1
 * if the hub went away. */
int hwemu_lockstep_wait(int handle, uint64_t* tick, uint64_t* micros);
/* Says this participant is done with `tick`. */
int hwemu_lockstep_ack(int handle, uint64_t tick);
void hwemu_lockstep_leave(int handle);

#ifdef __cplusplus
}
#endif

#endif /* HWEMU_CLIENT_H_ */
