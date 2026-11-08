#!/usr/bin/env python3
"""Host timeout recovery policy; no ROS executor or physical serial device needed."""
import os
import pty
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from esp32_bridge import Esp32Bridge, SerialPort, State


class BridgeHarness:
    recover_telemetry = Esp32Bridge.recover_telemetry
    fail = Esp32Bridge.fail
    feedback = Esp32Bridge.feedback

    def __init__(self):
        self.p = dict(telemetry_auto_recover=True, telemetry_recovery_duration=2.,
                      state_timeout=.6, stop_on_fault=True, auto_rearm=False)
        self.host_fault = 'TELEMETRY_TIMEOUT'
        self.telemetry_healthy_since = None
        self.transport_broken = self.software_estop = False
        self.state = SimpleNamespace(fault=0, estop=False, mode=2)
        self.reserved = False
        self.op = None
        self.armed = self.arming = self.auto_arm_pending = False
        self.command = (0., 0., 0.)
        self.await_command_zero = True
        self.logger = Mock()
        self.send = Mock(return_value=42)

    def get_logger(self):
        return self.logger

    def feed(self, times):
        previous = float('-inf')
        for now in times:
            self.recover_telemetry(now, previous, True)
            previous = now


class TelemetryRecoveryTest(unittest.TestCase):
    def test_fault_requests_stop_and_keeps_host_fault_latched(self):
        for reason in ['TELEMETRY_TIMEOUT', 'ROS_COMMAND_TIMEOUT', 'ARM_TIMEOUT',
                       'ACK_TIMEOUT', 'MCU_FAULT_8']:
            with self.subTest(reason=reason):
                b = BridgeHarness()
                b.host_fault = ''
                b.armed = True
                b.command = (1., 0., 0.)
                b.fail(reason)
                b.send.assert_called_once_with('D')
                self.assertEqual(b.host_fault, reason)
                self.assertFalse(b.armed or b.arming or b.auto_arm_pending)
                self.assertEqual(b.command, (0., 0., 0.))
                self.assertEqual(b.stop_seq, 42)
                self.assertEqual(b.hold_after_stop_seq, 42)
                self.assertFalse(b.handshake)
                b.fail(reason)
                b.send.assert_called_once()

    def test_broken_transport_does_not_attempt_serial_stop(self):
        b = BridgeHarness()
        b.host_fault = ''
        b.transport_broken = True
        b.fail('UART_READ: disconnected')
        b.send.assert_not_called()
        self.assertTrue(b.host_fault.startswith('UART_READ'))

    def test_stop_write_failure_does_not_recurse_or_clear_fault(self):
        b = BridgeHarness()
        b.host_fault = ''
        def write_failure(kind):
            b.transport_broken = True
            b.fail('UART_WRITE: disconnected')
            return None
        b.send.side_effect = write_failure
        b.fail('TELEMETRY_TIMEOUT')
        b.send.assert_called_once_with('D')
        self.assertTrue(b.host_fault.startswith('UART_WRITE'))
        b.feed([0, .5, 1, 1.5, 2, 2.5])
        self.assertTrue(b.host_fault.startswith('UART_WRITE'))

    def test_manual_cutoff_profile_can_disable_fault_stop(self):
        b = BridgeHarness()
        b.host_fault = ''
        b.p['stop_on_fault'] = False
        b.fail('TELEMETRY_TIMEOUT')
        b.send.assert_not_called()

    def test_stable_feedback_clears_without_arming_or_command_replay(self):
        b = BridgeHarness()
        b.feed([0, .5, 1, 1.5])
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')
        b.recover_telemetry(2, 1.5, True)
        self.assertEqual(b.host_fault, '')
        self.assertFalse(b.armed or b.arming or b.auto_arm_pending)
        self.assertEqual(b.command, (0., 0., 0.))
        self.assertTrue(b.await_command_zero)
        b.logger.info.assert_called_once()

    def test_gap_restarts_entire_stable_window(self):
        b = BridgeHarness()
        b.feed([0, .5, 1, 1.5, 2.2, 2.7, 3.2, 3.7])
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')
        b.recover_telemetry(4.2, 3.7, True)
        self.assertEqual(b.host_fault, '')

    def test_new_timeout_restarts_window(self):
        b = BridgeHarness()
        b.feed([0, .5, 1, 1.5])
        b.fail('TELEMETRY_TIMEOUT')
        b.recover_telemetry(2, 1.5, True)
        self.assertEqual(b.telemetry_healthy_since, 2)
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')

    def test_other_fault_supersedes_timeout_and_cannot_auto_clear(self):
        for reason in ['MCU_REBOOT; explicit stop and arm required', 'MCU_FAULT_8',
                       'MCU_ESTOP', 'UART_READ: disconnected', 'UART_WRITE: short write',
                       'ACK_TIMEOUT', 'ARM_TIMEOUT', 'ROS_COMMAND_TIMEOUT']:
            with self.subTest(reason=reason):
                b = BridgeHarness()
                b.feed([0, .5, 1])
                b.fail(reason)
                b.feed([2, 2.5, 3, 3.5, 4, 4.5])
                self.assertEqual(b.host_fault, reason)

    def test_unhealthy_inputs_prevent_recovery(self):
        for attribute in ['transport_broken', 'software_estop', 'reserved']:
            with self.subTest(attribute=attribute):
                b = BridgeHarness()
                setattr(b, attribute, True)
                b.feed([0, .5, 1, 1.5, 2, 2.5])
                self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')
        for key, value in [('fault', 8), ('estop', True), ('mode', 3), ('mode', 4)]:
            with self.subTest(key=key, value=value):
                b = BridgeHarness()
                setattr(b.state, key, value)
                b.feed([0, .5, 1, 1.5, 2, 2.5])
                self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')

    def test_changed_boot_cancels_window(self):
        b = BridgeHarness()
        b.feed([0, .5, 1, 1.5])
        b.recover_telemetry(2, 1.5, False)
        self.assertIsNone(b.telemetry_healthy_since)
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')

    def test_opt_out_keeps_fault_latched(self):
        b = BridgeHarness()
        b.p['telemetry_auto_recover'] = False
        b.feed([0, .5, 1, 1.5, 2, 2.5])
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')

    def test_rejected_duplicate_does_not_count_as_healthy_feedback(self):
        b = BridgeHarness()
        b.simple = True
        b.state = State(1, 10, 1000, 0, (0, 0, 0, 0), 0, 0, 2,
                        False, False, False, False, 1)
        b.received_at = 1.
        b.telemetry_healthy_since = .5
        b.invalid_frames = 0
        b.feedback(b.state)
        self.assertEqual(b.invalid_frames, 1)
        self.assertEqual(b.received_at, 1.)
        self.assertEqual(b.telemetry_healthy_since, .5)
        self.assertEqual(b.host_fault, 'TELEMETRY_TIMEOUT')


class ReopenHarness:
    maybe_reopen = Esp32Bridge.maybe_reopen

    def __init__(self, path):
        self.p = dict(port=path, baud=115200, serial_reopen_after=3.)
        self.serial = SerialPort(path, 115200)
        self.transport_broken = False
        self.serial_reopens = 0
        self.last_reopen_try = float('-inf')
        self.logger = Mock()
        self.fail = Mock()

    def get_logger(self):
        return self.logger


class SerialReopenTest(unittest.TestCase):
    """PTY thay CP2102: chỉ kiểm chứng logic đóng/mở lại, không phải lỗi USB thật."""
    def setUp(self):
        self.master, slave = pty.openpty()
        self.path = os.ttyname(slave)
        os.close(slave)
        self.b = ReopenHarness(self.path)

    def tearDown(self):
        self.b.serial.close()
        os.close(self.master)

    def test_recent_rx_keeps_port(self):
        old = self.b.serial
        self.b.maybe_reopen(old.last_rx + 1.)
        self.assertIs(self.b.serial, old)

    def test_silence_reopens_and_keeps_counters(self):
        old = self.b.serial
        os.write(self.master, b'S,1\n')
        self.assertEqual(len(old.read()), 1)
        self.b.maybe_reopen(old.last_rx + 3.5)
        self.assertIsNot(self.b.serial, old)
        self.assertEqual(old.fd, -1)
        self.assertEqual(self.b.serial.rx_lines, 1)
        self.assertEqual(self.b.serial_reopens, 1)
        self.assertFalse(self.b.transport_broken)
        os.write(self.master, b'S,2\n')
        self.assertEqual(len(self.b.serial.read()), 1)
        # Không mở lại liên tục: phải chờ thêm một chu kỳ.
        self.b.serial.last_rx = 0.
        self.b.maybe_reopen(self.b.last_reopen_try + 1.)
        self.assertEqual(self.b.serial_reopens, 1)

    def test_missing_port_faults_and_retries(self):
        self.b.p['port'] = self.path + '_missing'
        self.b.transport_broken = True
        self.b.maybe_reopen(100.)
        self.assertTrue(self.b.transport_broken)
        self.b.fail.assert_called_once()
        self.assertTrue(self.b.fail.call_args[0][0].startswith('UART_REOPEN'))
        self.b.p['port'] = self.path
        self.b.maybe_reopen(102.)
        self.assertEqual(self.b.serial_reopens, 0)
        self.b.maybe_reopen(103.)
        self.assertFalse(self.b.transport_broken)
        self.assertEqual(self.b.serial_reopens, 1)

    def test_zero_disables(self):
        self.b.p['serial_reopen_after'] = 0.
        old = self.b.serial
        self.b.maybe_reopen(old.last_rx + 100.)
        self.assertIs(self.b.serial, old)


class CommandHarness(BridgeHarness):
    recover_command = Esp32Bridge.recover_command
    rearm_after_recovery = Esp32Bridge.rearm_after_recovery

    def __init__(self):
        super().__init__()
        self.p.update(command_auto_recover=True, command_recovery_duration=1.,
                      command_timeout=.6, auto_rearm=True)
        self.host_fault = 'ROS_COMMAND_TIMEOUT'
        self.state = SimpleNamespace(fault=0, estop=False, mode=1)
        self.command_healthy_since = None
        self.auto_recoveries = 0
        self.input_zero = True
        self.cmd_at = 0.
        self.handshake = True
        self.handshake_seq = None
        self.is_fresh = self.is_stationary = True

    def fresh(self):
        return self.is_fresh

    def stationary(self):
        return self.is_stationary

    def confirmed(self, token, state=None):
        return False

    def feed(self, times):
        # /cmd_vel tươi đúng lúc kiểm tra (command_gate 50 Hz).
        for now in times:
            self.cmd_at = now
            self.recover_command(now)


class CommandRecoveryTest(unittest.TestCase):
    def test_zero_commands_clear_fault_and_request_rearm(self):
        b = CommandHarness()
        b.feed([0, .5])
        self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')
        b.feed([1.])
        self.assertEqual(b.host_fault, '')
        self.assertTrue(b.auto_arm_pending)
        self.assertFalse(b.armed or b.arming)
        self.assertEqual(b.command, (0., 0., 0.))
        self.assertEqual(b.auto_recoveries, 1)
        b.send.assert_not_called()  # MCU đã DISARMED + handshake: không cần D nữa

    def test_active_mcu_gets_stop_before_rearm(self):
        b = CommandHarness()
        b.state.mode = 2
        b.feed([0, 1.])
        b.send.assert_called_once_with('D')
        self.assertFalse(b.handshake)
        self.assertEqual(b.handshake_seq, 42)
        self.assertTrue(b.auto_arm_pending)

    def test_without_auto_rearm_stays_disarmed(self):
        b = CommandHarness()
        b.p['auto_rearm'] = False
        b.feed([0, 1.])
        self.assertEqual(b.host_fault, '')
        self.assertFalse(b.auto_arm_pending)

    def test_unsafe_conditions_keep_fault(self):
        cases = [('input_zero', False), ('is_fresh', False), ('is_stationary', False),
                 ('software_estop', True), ('transport_broken', True), ('reserved', True)]
        for attribute, value in cases:
            with self.subTest(attribute=attribute):
                b = CommandHarness()
                setattr(b, attribute, value)
                b.feed([0, .5, 1., 1.5, 2.])
                self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')
                self.assertFalse(b.auto_arm_pending)
        for key, value in [('fault', 8), ('estop', True), ('mode', 3), ('mode', 4)]:
            with self.subTest(key=key, value=value):
                b = CommandHarness()
                setattr(b.state, key, value)
                b.feed([0, .5, 1., 1.5, 2.])
                self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')

    def test_nonzero_command_restarts_window(self):
        b = CommandHarness()
        b.feed([0, .5])
        b.input_zero = False
        b.feed([.7])
        b.input_zero = True
        b.feed([.9, 1.5])
        self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')
        b.feed([2.])
        self.assertEqual(b.host_fault, '')

    def test_stale_command_does_not_count(self):
        b = CommandHarness()
        b.cmd_at = 0.
        for now in [0, .5, 1., 1.5]:
            b.recover_command(now)  # không có /cmd_vel mới
        self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')

    def test_other_faults_never_auto_clear(self):
        for reason in ['ACK_TIMEOUT', 'ARM_TIMEOUT', 'MCU_FAULT_8', 'MCU_ESTOP',
                       'MCU_REBOOT; explicit stop and arm required', 'TELEMETRY_TIMEOUT']:
            with self.subTest(reason=reason):
                b = CommandHarness()
                b.host_fault = reason
                b.feed([0, .5, 1., 1.5, 2.])
                self.assertEqual(b.host_fault, reason)

    def test_opt_out_keeps_fault_latched(self):
        b = CommandHarness()
        b.p['command_auto_recover'] = False
        b.feed([0, .5, 1., 1.5, 2.])
        self.assertEqual(b.host_fault, 'ROS_COMMAND_TIMEOUT')

    def test_telemetry_recovery_also_rearms_when_enabled(self):
        b = BridgeHarness()
        b.p['auto_rearm'] = True
        b.handshake, b.handshake_seq, b.auto_recoveries = True, None, 0
        b.state.mode = 1
        b.rearm_after_recovery = lambda reason: CommandHarness.rearm_after_recovery(b, reason)
        b.confirmed = lambda token, state=None: False
        b.feed([0, .5, 1, 1.5, 2])
        self.assertEqual(b.host_fault, '')
        self.assertTrue(b.auto_arm_pending)


class ExplicitStopRearmTest(unittest.TestCase):
    """03/11/2026: /lift/stop sau mission FAULT xóa lỗi timeout thì phải tự arm lại."""
    def make(self, fault, rearm=True):
        b = CommandHarness()
        b.explicit_stop = Esp32Bridge.explicit_stop.__get__(b)
        b.host_fault = fault;b.p['auto_rearm'] = rearm
        b.stop_generation = 0;b.height = 0.;b.hold_height = 0.;b.cmd_at = 0.
        b.state = SimpleNamespace(fault=0, estop=False, mode=1, homed=True)
        return b

    def test_lift_stop_clearing_timeout_rearms(self):
        for fault in ('ROS_COMMAND_TIMEOUT', 'TELEMETRY_TIMEOUT'):
            with self.subTest(fault=fault):
                b = self.make(fault);b.explicit_stop()
                self.assertEqual(b.host_fault, '');self.assertTrue(b.auto_arm_pending)

    def test_operator_disarm_or_other_fault_does_not_rearm(self):
        b = self.make('ROS_COMMAND_TIMEOUT');b.explicit_stop(disarm=True);self.assertFalse(b.auto_arm_pending)
        b = self.make('ACK_TIMEOUT');b.explicit_stop();self.assertFalse(b.auto_arm_pending)
        b = self.make('ROS_COMMAND_TIMEOUT', rearm=False);b.explicit_stop();self.assertFalse(b.auto_arm_pending)


if __name__ == '__main__':
    unittest.main()
