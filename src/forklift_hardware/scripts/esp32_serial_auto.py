# SPDX-License-Identifier: Apache-2.0
"""Identify the forklift MCU by passive telemetry, without sending commands."""
from glob import glob
import os
import shutil
import subprocess
import time


def usb_serial_candidates():
    # Prefer stable names, but include tty names for boards without USB serial IDs.
    paths = sorted(glob('/dev/serial/by-id/*'))
    paths += sorted(glob('/dev/ttyUSB*') + glob('/dev/ttyACM*'))
    seen = set()
    result = []
    for path in paths:
        device = os.path.realpath(path)
        if device not in seen:
            seen.add(device)
            result.append(path)
    return result


def port_in_use(path):
    if not shutil.which('fuser'):
        raise RuntimeError('port=auto needs fuser (sudo apt install psmisc); '
                           'alternatively specify port:=/dev/serial/by-id/...')
    result = subprocess.run(['fuser', '-s', path], timeout=2, check=False)
    if result.returncode not in (0, 1):
        raise RuntimeError(f'Cannot check whether serial port is busy: {path}')
    return result.returncode == 0


def detect_esp32(baud, parser, port_factory, log, probe_seconds=3.0):
    """Return the unique MCU's (path, open port); close every other probe.

    Two valid, advancing telemetry timestamps identify a candidate. Keeping the
    selected descriptor open avoids resetting/reopening it after detection.
    Never pick the first tty or identify an ESP32 solely by its USB chip ID.
    """
    matches = []
    try:
        for path in usb_serial_candidates():
            if port_in_use(path):
                log(f'Auto serial: skipping busy port {path}')
                continue
            port = None
            keep = False
            try:
                log(f'Auto serial: listening for ESP32 telemetry on {path}')
                port = port_factory(path, baud)
                deadline = time.monotonic() + probe_seconds
                previous_us = None
                while time.monotonic() < deadline:
                    for line in port.read():
                        try:
                            state = parser(line)
                        except (ValueError, UnicodeError):
                            continue
                        if previous_us is not None and state.us > previous_us:
                            matches.append((path, port))
                            keep = True
                            break
                        previous_us = state.us
                    if keep:
                        break
                    time.sleep(.02)
            except OSError as error:
                log(f'Auto serial: cannot probe {path}: {error}')
            finally:
                if port is not None and not keep:
                    port.close()
        if len(matches) != 1:
            if matches:
                raise RuntimeError('Multiple ESP32 firmware devices found: '
                                   + ', '.join(path for path, _ in matches)
                                   + '; specify port:= explicitly')
            raise RuntimeError('No free USB port with valid ESP32 telemetry found. '
                               'Connect ESP32, stop its old bridge/Serial Monitor, '
                               'check serial permissions and restart launch; '
                               'or specify port:= explicitly.')
        return matches[0]
    except BaseException:
        for _, port in matches:
            port.close()
        raise
