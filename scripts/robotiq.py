#!/usr/bin/env python

import time
import struct
import socket
import threading


class robotiq():
    """Manipulates robotiq gripper via UR controller."""
    def __init__(self):
        self.sock = None
        self._cont = False
        self._sem = None
        self._status_updater = None
        self._max_position = 255
        self._min_position = 0

    def _update_status(self):
        """Updates gripper's current status."""
        while self._cont:
            self.status()
            time.sleep(0.5)

    def connect(self, ip, port):
        """Open a socket connection via RS485."""
        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))
        self._cont = True
        self._sem = threading.Semaphore(1)
        self._status_updater = threading.Thread(
            target=self._update_status)
        self._status_updater.start()

    def disconnect(self):
        """Disconnects the socket connection opened."""
        self._cont = False
        self._status_updater.join()
        self._status_updater = None
        self.sock.close()
        self.sock = None
        self._sem = None

    def _calc_crc(self, command):
        """Generates checksum."""
        crc_registor = 0xFFFF
        for data_byte in command:
            tmp = crc_registor ^ data_byte
            for _ in range(8):
                if(tmp & 1 == 1):
                    tmp = tmp >> 1
                    tmp = 0xA001 ^ tmp
                else:
                    tmp = tmp >> 1
            crc_registor = tmp
        crc = bytearray(struct.pack('<H', crc_registor))
        return crc

    def send_command(self, command):
        """Sends a command."""
        with self._sem:
            crc = self._calc_crc(command)
            data = command + crc
            self.sock.sendall(data)
            time.sleep(0.001)
            data = self.sock.recv(1024)
        return bytearray(data)

    def status(self):
        """Check status."""
        command = bytearray(
            b'\x09\x03\x07\xD0\x00\x03')
        return self.send_command(command)

    def reset(self):
        """Resets gripper."""
        command = bytearray(
            b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def activate(self):
        """Activates gripper."""
        command = bytearray(
            b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def wait_activate_complete(self):
        """Waits for completing activation."""
        while True:
            data = self.status()
            if data[5] != 0x00:
                return data[3]
            if data[3] == 0x31 and data[7] < 4:
                return data[3]

    def adjust(self):
        """Calibrates gripper's open-close."""
        self.move(255, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._max_position = position
        self.move(0, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._min_position = position

    def get_position_mm(self, position):
        """Gets position of the fingers."""
        if position > self._max_position:
            position = self._max_position
        elif position < self._min_position:
            position = self._min_position
        position_mm = 85.0 * \
            (self._max_position - position) / \
            (self._max_position - self._min_position)
        return position_mm

    def get_force_mA(self, force):
        return 10.0 * force

    # position: 0x00...open, 0xff...close
    # speed: 0x00...minimum, 0xff...maximum
    # force: 0x00...minimum, 0xff...maximum
    def move(self, position, speed, force):
        """Moves with parameters."""
        command = bytearray(
            b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\x00\x00')
        command[10] = position
        command[11] = speed
        command[12] = force
        return self.send_command(command)

    # result: (status, position, force)
    def wait_move_complete(self):
        """Waits for completing movements."""
        while True:
            data = self.status()
            if data[5] != 0x00:
                return (-1, data[7], data[8])
            if data[3] == 0x79:
                return (2, data[7], data[8])
            if data[3] == 0xb9:
                return (1, data[7], data[8])
            if data[3] == 0xf9:
                return (0, data[7], data[8])

