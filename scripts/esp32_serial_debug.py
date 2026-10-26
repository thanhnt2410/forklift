#!/usr/bin/env python3
"""Inspect simple ESP32 telemetry, or explicitly test homing, without ROS."""
import argparse
import time

import serial


MODES = ('LOCKED', 'DISARMED', 'ACTIVE', 'HOMING', 'FAULTED')
FAULTS = {1: 'CONFIG', 2: 'ESTOP', 4: 'WATCHDOG', 8: 'ENCODER',
          16: 'LIMIT', 32: 'HOME_TIMEOUT_OR_STEP_BUDGET', 64: 'DEADLINE',
          128: 'PERIPHERAL'}
HEARTBEAT = b'C,0,0,0,0,0,0\n'


def parse(line):
    fields = line.decode('ascii').strip().split(',')
    if len(fields) != 12 or fields[0] != 'S':
        raise ValueError('expected simple S telemetry with 12 fields')
    values = tuple(map(int, fields[1:]))
    ms, fl, fr, rl, rr, steps, mode, homed, bottom, fault, caps = values
    if (ms < 0 or mode not in range(5) or homed not in (0, 1)
            or bottom not in (0, 1) or fault not in range(256) or caps not in (0, 1, 3)):
        raise ValueError('invalid telemetry values')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='COM8, /dev/ttyUSB0, etc.')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--home', action='store_true', help='physically run lift homing')
    parser.add_argument('--reset', action='store_true', help='stop and clear faults before homing')
    args = parser.parse_args()
    if args.reset and not args.home:
        parser.error('--reset requires --home')
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.02, write_timeout=0.2)
    port.dtr = port.rts = False
    port.port = args.port
    actuating = False
    try:
        port.open()
        print(f'{args.port} @ {args.baud}; ' +
              ('HOME requested; wheel targets stay zero.' if args.home else
               'MONITOR ONLY: no commands sent. Press/release the bottom switch.'), flush=True)
        opened = last_valid = time.monotonic()
        deadline = opened + 40.
        last_heartbeat = 0.
        phase = 'waiting'
        last_status = None
        buffer = bytearray()
        while True:
            now = time.monotonic()
            if actuating and now - last_heartbeat >= 0.1:
                port.write(HEARTBEAT)
                last_heartbeat = now
            # Bound both reads and retained data even if the wire contains no LF.
            buffer.extend(port.read(min(max(port.in_waiting, 1), 512)))
            while b'\n' in buffer:
                line, _, tail = buffer.partition(b'\n')
                buffer = bytearray(tail)
                try:
                    state = parse(line)
                except (ValueError, UnicodeError):
                    print(f'INVALID RX: {bytes(line[:96])!r}', flush=True)
                    continue
                last_valid = time.monotonic()
                ms, fl, fr, rl, rr, steps, mode, homed, bottom, fault, caps = state
                status = state[5:]
                if status != last_status:
                    reason = '|'.join(name for bit, name in FAULTS.items() if fault & bit) or 'NONE'
                    print(f'ms={ms} mode={MODES[mode]} homed={homed} bottom={bottom} '
                          f'steps={steps} fault={fault}({reason}) caps={caps}', flush=True)
                    last_status = status
                if not args.home:
                    continue
                # Allow USB-open/reset transients to settle before sending anything.
                if phase == 'waiting' and now - opened >= 2.:
                    if caps != 3:
                        raise RuntimeError('Firmware does not report wheel+lift capability')
                    if fault and not args.reset:
                        raise RuntimeError('Fault present; diagnose it, then use --home --reset')
                    actuating = True
                    port.write(b'D\n' + (b'R\n' if args.reset else b'') + HEARTBEAT)
                    phase = 'stopping'
                elif phase == 'stopping' and mode == 1 and fault == 0:
                    port.write(b'H\n')
                    phase = 'starting'
                    print('H sent; waiting for HOMING.', flush=True)
                elif phase == 'starting' and mode == 3:
                    phase = 'homing'
                elif phase == 'homing' and homed == 1 and mode == 1:
                    print('HOME SUCCESS: homed=1', flush=True)
                    return 0
                if phase in ('starting', 'homing') and fault:
                    raise RuntimeError(f'Home failed with fault={fault}; inspect status above')
            if len(buffer) > 512:
                print(f'NO NEWLINE: first bytes={bytes(buffer[:64])!r}', flush=True)
                buffer.clear()
            if now - last_valid > (3. if actuating else 5.):
                raise RuntimeError('No recent valid telemetry; verify firmware, baud and USB port')
            if args.home and now >= deadline:
                raise RuntimeError('Home did not complete within 40 seconds')
    except KeyboardInterrupt:
        return 0
    except (OSError, serial.SerialException, RuntimeError) as error:
        print(f'ERROR: {error}', flush=True)
        return 1
    finally:
        if port.is_open:
            try:
                if actuating:
                    port.write(b'D\n')
                    port.flush()
            finally:
                port.close()


if __name__ == '__main__':
    raise SystemExit(main())
