#!/usr/bin/env python3

"""AMR-local NTRIP client that publishes RTCM messages reliably."""

from __future__ import annotations

import base64
import datetime
import socket
import ssl
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtcm_msgs.msg import Message
from sensor_msgs.msg import NavSatFix, NavSatStatus


class AmrNtripClient(Node):
    """Minimal NTRIP client for RTCM streaming with optional GGA uplink."""

    def __init__(self) -> None:
        super().__init__('ntrip_client')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '127.0.0.1'),
                ('port', 2101),
                ('mountpoint', 'mount'),
                ('ntrip_version', 'Ntrip/2.0'),
                ('authenticate', False),
                ('username', ''),
                ('password', ''),
                ('ssl', False),
                ('cert', 'None'),
                ('key', 'None'),
                ('ca_cert', 'None'),
                ('rtcm_frame_id', 'gnss_link'),
                ('failed_connection_retry_seconds', 5.0),
                ('reconnect_attempt_wait_seconds', 5),
                ('socket_timeout_seconds', 10.0),
                ('rtcm_timeout_seconds', 4.0),
                ('retry_attempts_before_error', 3),
                ('send_nmea', False),
            ],
        )

        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        self._mountpoint = str(self.get_parameter('mountpoint').value)
        self._ntrip_version = str(self.get_parameter('ntrip_version').value)
        self._authenticate = bool(self.get_parameter('authenticate').value)
        self._username = str(self.get_parameter('username').value)
        self._password = str(self.get_parameter('password').value)
        self._use_ssl = bool(self.get_parameter('ssl').value)
        self._cert = self._none_if_literal_none(str(self.get_parameter('cert').value))
        self._key = self._none_if_literal_none(str(self.get_parameter('key').value))
        self._ca_cert = self._none_if_literal_none(str(self.get_parameter('ca_cert').value))
        self._rtcm_frame_id = str(self.get_parameter('rtcm_frame_id').value)
        self._reconnect_wait = float(self.get_parameter('reconnect_attempt_wait_seconds').value)
        self._failed_connection_retry = float(
            self.get_parameter('failed_connection_retry_seconds').value
        )
        self._socket_timeout = float(self.get_parameter('socket_timeout_seconds').value)
        self._rtcm_timeout = float(self.get_parameter('rtcm_timeout_seconds').value)
        self._retry_attempts_before_error = int(
            self.get_parameter('retry_attempts_before_error').value
        )
        self._send_nmea = bool(self.get_parameter('send_nmea').value)

        if self._authenticate and (not self._username or not self._password):
            raise ValueError(
                'authenticate is true, but username or password is empty'
            )

        if self._failed_connection_retry <= 0.0:
            self._failed_connection_retry = self._reconnect_wait
        if self._retry_attempts_before_error < 1:
            self._retry_attempts_before_error = 1

        self._publisher = self.create_publisher(Message, 'rtcm', 10)
        self._socket_lock = threading.Lock()
        self._socket: Optional[socket.socket] = None
        self._stop_event = threading.Event()
        self._parser_buffer = bytearray()
        self._last_valid_rtcm_time: Optional[float] = None
        self._connection_issue_count = 0
        self._bad_rtcm_issue_count = 0

        if self._send_nmea:
            best_effort_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self._fix_subscription = self.create_subscription(
                NavSatFix,
                'fix',
                self._handle_fix,
                best_effort_qos,
            )
        else:
            self._fix_subscription = None

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    @staticmethod
    def _none_if_literal_none(value: str) -> Optional[str]:
        return None if value == 'None' else value

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._close_socket()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._connect_and_stream()
            except Exception as exc:  # pragma: no cover
                self._report_connection_issue(
                    f'NTRIP reconnect after error: {exc}. '
                    f'Retrying in {self._failed_connection_retry:.1f}s'
                )
                self._close_socket()
            if not self._stop_event.is_set():
                time.sleep(self._failed_connection_retry)

    def _connect_and_stream(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self._socket_timeout)
        sock.connect((self._host, self._port))

        if self._use_ssl:
            context = ssl.create_default_context()
            if self._cert:
                context.load_cert_chain(self._cert, self._key)
            if self._ca_cert:
                context.load_verify_locations(self._ca_cert)
            sock = context.wrap_socket(sock, server_hostname=self._host)
            sock.settimeout(self._socket_timeout)

        sock.sendall(self._build_request())
        response = sock.recv(4096)
        if not response:
            raise RuntimeError('No response received from NTRIP caster')

        header, payload = self._split_response(response)
        if not any(code in header for code in ('ICY 200 OK', 'HTTP/1.0 200 OK', 'HTTP/1.1 200 OK')):
            raise RuntimeError(f'Invalid caster response: {header.strip()}')

        with self._socket_lock:
            self._socket = sock

        self._parser_buffer.clear()
        self._last_valid_rtcm_time = None
        self.get_logger().info(
            f'Connected to http://{self._host}:{self._port}/{self._mountpoint}'
        )

        if payload:
            self._handle_rtcm_bytes(payload)

        while not self._stop_event.is_set():
            try:
                chunk = sock.recv(4096)
            except socket.timeout as exc:
                if self._stream_timed_out():
                    raise RuntimeError(
                        f'No valid RTCM received for {self._rtcm_timeout:.1f}s'
                    ) from exc
                continue
            if not chunk:
                raise RuntimeError('Socket closed by caster')
            self._handle_rtcm_bytes(chunk)

    def _build_request(self) -> bytes:
        lines = [f'GET /{self._mountpoint} HTTP/1.0']
        if self._ntrip_version and self._ntrip_version != 'None':
            lines.append(f'Ntrip-Version: {self._ntrip_version}')
        lines.append('User-Agent: NTRIP amr_sweeper_gnss')
        if self._authenticate:
            auth = base64.b64encode(
                f'{self._username}:{self._password}'.encode('utf-8')
            ).decode('utf-8')
            lines.append(f'Authorization: Basic {auth}')
        return ('\r\n'.join(lines) + '\r\n\r\n').encode('utf-8')

    def _split_response(self, response: bytes) -> tuple[str, bytes]:
        marker = b'\r\n\r\n'
        if marker in response:
            header_bytes, payload = response.split(marker, 1)
            return header_bytes.decode('ISO-8859-1', errors='replace'), payload
        return response.decode('ISO-8859-1', errors='replace'), b''

    def _handle_rtcm_bytes(self, chunk: bytes) -> None:
        self._parser_buffer.extend(chunk)
        while True:
            packet = self._extract_rtcm_packet()
            if packet is None:
                break
            if not self._validate_rtcm_packet(packet):
                continue
            msg = Message()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._rtcm_frame_id
            msg.message = list(packet)
            self._publisher.publish(msg)
            self._last_valid_rtcm_time = time.monotonic()
            self._reset_issue_counters()

    def _extract_rtcm_packet(self) -> Optional[bytes]:
        while self._parser_buffer and self._parser_buffer[0] != 0xD3:
            del self._parser_buffer[0]

        if len(self._parser_buffer) < 3:
            return None

        payload_len = ((self._parser_buffer[1] & 0x03) << 8) | self._parser_buffer[2]
        total_len = 3 + payload_len + 3
        if len(self._parser_buffer) < total_len:
            return None

        packet = bytes(self._parser_buffer[:total_len])
        del self._parser_buffer[:total_len]
        return packet

    def _validate_rtcm_packet(self, packet: bytes) -> bool:
        payload = packet[3:-3]
        if not payload:
            self._report_bad_rtcm_issue('Discarding empty RTCM packet')
            return False
        if all(byte == 0 for byte in payload):
            self._report_bad_rtcm_issue('Discarding RTCM packet with all-zero payload')
            return False
        expected_crc = int.from_bytes(packet[-3:], byteorder='big')
        actual_crc = self._crc24q(packet[:-3])
        if expected_crc != actual_crc:
            self._report_bad_rtcm_issue(
                f'Discarding RTCM packet with failed CRC '
                f'(expected=0x{expected_crc:06X}, actual=0x{actual_crc:06X})'
            )
            return False
        return True

    def _handle_fix(self, fix: NavSatFix) -> None:
        sentence = self._build_gga_sentence(fix)
        with self._socket_lock:
            sock = self._socket
        if sock is None:
            return
        try:
            sock.sendall(sentence.encode('utf-8'))
        except OSError as exc:
            self.get_logger().warning(f'Unable to send NMEA sentence to server: {exc}')
            self._close_socket()

    def _build_gga_sentence(self, fix: NavSatFix) -> str:
        timestamp_secs = fix.header.stamp.sec + fix.header.stamp.nanosec * 1e-9
        timestamp = datetime.datetime.fromtimestamp(timestamp_secs, datetime.UTC).time()
        nmea_utc = (
            f'{timestamp.hour:02}{timestamp.minute:02}{timestamp.second:02}.'
            f'{int(timestamp.microsecond * 1e-4):02}'
        )

        lat_dir = 'N' if fix.latitude >= 0.0 else 'S'
        lon_dir = 'E' if fix.longitude >= 0.0 else 'W'
        lat = self._dd_to_dmm(abs(fix.latitude), True)
        lon = self._dd_to_dmm(abs(fix.longitude), False)

        if fix.status.status == NavSatStatus.STATUS_FIX:
            quality = 1
        elif fix.status.status == NavSatStatus.STATUS_SBAS_FIX:
            quality = 2
        elif fix.status.status == NavSatStatus.STATUS_GBAS_FIX:
            quality = 5
        else:
            quality = 0

        body = (
            f'$GPGGA,{nmea_utc},{lat},{lat_dir},{lon},{lon_dir},'
            f'{quality},05,1.0,{fix.altitude:.1f},M,-32.0,M,,0000'
        )
        return f'{body}*{self._nmea_checksum(body):02X}\r\n'

    @staticmethod
    def _dd_to_dmm(value: float, is_lat: bool) -> str:
        degrees = int(value)
        minutes = (value - degrees) * 60.0
        width = 2 if is_lat else 3
        return f'{degrees:0{width}d}{minutes:07.4f}'

    @staticmethod
    def _nmea_checksum(sentence: str) -> int:
        checksum = 0
        for char in sentence[1:]:
            checksum ^= ord(char)
        return checksum

    def _close_socket(self) -> None:
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _stream_timed_out(self) -> bool:
        if self._rtcm_timeout <= 0.0:
            return False
        if self._last_valid_rtcm_time is None:
            return False
        return (time.monotonic() - self._last_valid_rtcm_time) >= self._rtcm_timeout

    def _report_connection_issue(self, message: str) -> None:
        self._connection_issue_count += 1
        self._log_escalating_issue(self._connection_issue_count, message)

    def _report_bad_rtcm_issue(self, message: str) -> None:
        self._bad_rtcm_issue_count += 1
        self._log_escalating_issue(self._bad_rtcm_issue_count, message)

    def _log_escalating_issue(self, count: int, message: str) -> None:
        if count < self._retry_attempts_before_error:
            self.get_logger().warning(message)
            return
        if count == self._retry_attempts_before_error:
            self.get_logger().error(
                f'{message}. Escalating after {count} consecutive failures'
            )
            return
        self.get_logger().error(message)

    def _reset_issue_counters(self) -> None:
        self._connection_issue_count = 0
        self._bad_rtcm_issue_count = 0

    @staticmethod
    def _crc24q(data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= byte << 16
            for _ in range(8):
                crc <<= 1
                if crc & 0x1000000:
                    crc ^= 0x1864CFB
        return crc & 0xFFFFFF


def main() -> None:
    rclpy.init()
    node = AmrNtripClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
