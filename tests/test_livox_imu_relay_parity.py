"""With Humble and the colcon overlay sourced, run via pytest or --bench."""

import argparse
from contextlib import ExitStack, contextmanager
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
from tempfile import TemporaryFile
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


SOURCE_RELAY = (Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'
                / 'yahboomcar_bringup' / 'livox_imu_relay.py')
RATE_HZ = 200
INPUT_TOPIC = '/imu_relay_test/livox'
CPP_PACKAGE = 'yahboomcar_imu_relay_cpp'
CPP_EXECUTABLE = 'livox_imu_relay_cpp'


def _commands():
    prefix = Path(get_package_prefix(CPP_PACKAGE))
    return {
        'py': [sys.executable, str(SOURCE_RELAY)],
        'cpp': [str(prefix / 'lib' / CPP_PACKAGE / CPP_EXECUTABLE)],
    }


def _isolate_ros(setenv, unsetenv):
    setenv('ROS_LOCALHOST_ONLY', '1')
    setenv('ROS_DOMAIN_ID', str(random.randrange(60, 100)))
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE'):
        unsetenv(name)


@contextmanager
def _relay(command, impl, args):
    with TemporaryFile(mode='w+t') as log:
        process = subprocess.Popen([
            *command, '--ros-args', '-r', f'__node:=test_relay_{impl}',
            *args,
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            yield process
            assert process.poll() is None, f'{impl} relay exited early'
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def _spin_until(node, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not condition():
        remaining = deadline - time.monotonic()
        assert remaining > 0, 'Timed out waiting for relay messages or DDS discovery'
        rclpy.spin_once(node, timeout_sec=min(remaining, 0.05))


@contextmanager
def _harness(commands, args=(), *, override_topics=False):
    rclpy.init()
    try:
        node = rclpy.create_node('imu_relay_test')
        try:
            publisher = node.create_publisher(Imu, INPUT_TOPIC, qos_profile_sensor_data)
            received = {impl: [] for impl in commands}
            with ExitStack() as stack:
                processes, topics = {}, []
                for impl, command in commands.items():
                    topic = f'/imu_relay_test/{impl}'
                    topics.append(topic)
                    node.create_subscription(Imu, topic, received[impl].append, 50)
                    if override_topics:
                        topic_args = ['-p', f'input_topic:={INPUT_TOPIC}', '-p', f'output_topic:={topic}']
                    else:
                        topic_args = ['-r', f'/livox/imu:={INPUT_TOPIC}', '-r', f'imu/data_raw:={topic}']
                    processes[impl] = stack.enter_context(_relay(command, impl, [*topic_args, *args]))
                _spin_until(node, lambda: (
                    publisher.get_subscription_count() == len(commands)
                    and all(node.count_publishers(topic) == 1 for topic in topics)
                ))
                yield node, publisher, received, processes
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def _sample(index):
    window = index // RATE_HZ
    sign = (-1.0) ** index
    gyro_z = [0.003, 0.003, 0.008, 0.008, 0.010, 0.010, 0.025, 0.010, 0.010]
    msg = Imu()
    msg.header.frame_id = 'incoming_frame'
    msg.header.stamp.sec = 100 + index // RATE_HZ
    msg.header.stamp.nanosec = index * (1_000_000_000 // RATE_HZ) % 1_000_000_000
    msg.orientation.x = 0.5
    msg.orientation_covariance[0] = 1.0
    msg.angular_velocity_covariance[0] = 2.0
    msg.linear_acceleration_covariance[0] = 3.0
    msg.angular_velocity.x = -0.002
    msg.angular_velocity.y = 0.001
    msg.angular_velocity.z = gyro_z[window % len(gyro_z)] + sign * (0.04 if window == 3 else 0.0004)
    msg.linear_acceleration.x = 0.01
    msg.linear_acceleration.y = -0.02
    msg.linear_acceleration.z = 1.0 + (sign * 0.04 if window == 5 else 0.0)
    return msg


def _assert_equal(py, cpp):
    assert py.header == cpp.header
    for name, axes in (('orientation', 'xyzw'), ('angular_velocity', 'xyz'),
                       ('linear_acceleration', 'xyz')):
        expected = [getattr(getattr(py, name), axis) for axis in axes]
        actual = [getattr(getattr(cpp, name), axis) for axis in axes]
        assert actual == pytest.approx(expected, rel=0, abs=1e-9), name
        covariance = name + '_covariance'
        assert list(getattr(cpp, covariance)) == pytest.approx(
            list(getattr(py, covariance)), rel=0, abs=1e-9), covariance


@pytest.mark.parametrize('restamp', [False, True])
def test_livox_imu_relay_parity(monkeypatch, restamp):
    try:
        commands = _commands()
    except PackageNotFoundError:
        pytest.skip('Build and source yahboomcar_imu_relay_cpp to run the ROS parity test')
    _isolate_ros(monkeypatch.setenv, lambda name: monkeypatch.delenv(name, raising=False))
    if restamp:
        args = ['-p', 'use_sim_time:=true', '-p', 'track_bias:=false',
                '-p', 'frame_id:=custom_imu', '-p', 'gyro_var:=0.0003', '-p', 'accel_var:=0.023']
    else:
        args = ['-p', 'restamp:=false']

    with _harness(commands, args, override_topics=restamp) as (node, publisher, received, _):
        if restamp:
            clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
            _spin_until(node, lambda: clock.get_subscription_count() == len(commands))
            # Both clock sources must observe the same nonzero time before sampling.
            tick = Clock()
            tick.clock.sec, tick.clock.nanosec = 1234, 567
            for attempt in range(20):
                clock.publish(tick)
                publisher.publish(_sample(attempt))
                _spin_until(node, lambda: all(messages for messages in received.values()))
                if all(messages[-1].header.stamp == tick.clock for messages in received.values()):
                    break
                for messages in received.values():
                    messages.clear()
            else:
                pytest.fail('Both relay clocks did not select simulated time')
            for messages in received.values():
                messages.clear()

        # Use one input in flight so best-effort DDS cannot skip bias samples.
        for index in range(RATE_HZ * (3 if restamp else 9)):
            sample = _sample(index)
            publisher.publish(sample)
            _spin_until(node, lambda: all(len(messages) > index for messages in received.values()))
            py, cpp = (received[impl][index] for impl in ('py', 'cpp'))
            _assert_equal(py, cpp)
            assert cpp.header.stamp == (tick.clock if restamp else sample.header.stamp)
            if restamp:
                assert cpp.angular_velocity == sample.angular_velocity

        if restamp:
            assert cpp.header.frame_id == 'custom_imu'
            assert cpp.angular_velocity_covariance[0] == 0.0003
            assert cpp.linear_acceleration_covariance[0] == 0.023
        else:
            assert cpp.header.frame_id == 'livox_frame'
            assert cpp.angular_velocity_covariance[0] == 1e-4
            assert cpp.linear_acceleration_covariance[0] == 1e-2
            # First snap, gain on the next quiet window, bias held during
            # motion, and recovery after two consecutive quiet windows.
            for index, bias in [(398, 0), (399, 0.003), (599, 0.004), (799, 0.004),
                                (999, 0.004), (1199, 0.004), (1399, 0.004), (1599, 0.004),
                                (1799, 0.0052)]:
                raw = _sample(index).angular_velocity.z
                actual = received['cpp'][index].angular_velocity.z
                assert actual == pytest.approx(raw - bias, rel=0, abs=1e-9)
        assert cpp.orientation_covariance[0] == -1
        assert cpp.linear_acceleration.z == 9.80665


def _cpu_seconds(pid):
    # Strip comm in parentheses; utime and stime are fields 14 and 15.
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return sum(map(int, fields[11:13])) / os.sysconf('SC_CLK_TCK')


def _stream(node, publisher, duration):
    start = time.monotonic()
    for index in range(duration * RATE_HZ + 1):
        remaining = start + index / RATE_HZ - time.monotonic()
        while remaining > 0:
            rclpy.spin_once(node, timeout_sec=remaining)
            remaining = start + index / RATE_HZ - time.monotonic()
        if index < duration * RATE_HZ:
            publisher.publish(_sample(index % (9 * RATE_HZ)))
    return time.monotonic() - start


def _benchmark(seconds):
    _isolate_ros(os.environ.__setitem__, lambda name: os.environ.pop(name, None))
    for impl, command in _commands().items():
        with _harness({impl: command}) as (node, publisher, received, processes):
            outputs = received[impl]
            _stream(node, publisher, 3)
            warmup_count = len(outputs)
            pid = processes[impl].pid
            before = _cpu_seconds(pid)
            elapsed = _stream(node, publisher, seconds)
            used = _cpu_seconds(pid) - before
            sent = seconds * RATE_HZ
            count = len(outputs) - warmup_count
            print(f'{impl}: {100 * used / elapsed:.2f}% of one core, '
                  f'{count}/{sent} received ({count / elapsed:.1f} Hz) in {elapsed:.2f} s')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Benchmark the Livox IMU relays')
    parser.add_argument('--bench', action='store_true', required=True)
    parser.add_argument('--seconds', type=int, default=20, help='Measurement duration per relay')
    bench_args = parser.parse_args()
    if bench_args.seconds <= 0:
        parser.error('--seconds must be positive')
    _benchmark(bench_args.seconds)
