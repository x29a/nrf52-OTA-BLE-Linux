#!/usr/bin/env python3.8

# DFU Server for Nordic nRF52 based systems
# Conforms to nRF52_SDK 11.0 BLE_DFU requirements

import argparse
import binascii
import logging
import logging.config
import sys
import time
import zipfile

import bluepy.btle

from array import array

LOG = logging.getLogger(__name__)
LOG_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'local': {
            'format': '%(asctime)s %(message)s',
        }
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'local',
        }
    },
    'root': {
        'level': 'DEBUG',
        'handlers': ['console'],
       }
}

# DFU Opcodes
START_DFU = 1
INITIALIZE_DFU = 2
RECEIVE_FIRMWARE_IMAGE = 3
VALIDATE_FIRMWARE_IMAGE = 4
ACTIVATE_FIRMWARE_AND_RESET = 5
SYSTEM_RESET = 6
PKT_RCPT_NOTIF_REQ = 8


DFU_PROCEDURE = {
    "01": "START",
    "02": "INIT",
    "03": "RECEIVE_APP",
    "04": "VALIDATE",
    "08": "PKT_RCPT_REQ",
}

DFU_OPERATION = {
    "01": "START_DFU",
    "02": "RECEIVE_INIT",
    "03": "RECEIVE_FW",
    "04": "VALIDATE",
    "05": "ACTIVATE_N_RESET",
    "06": "SYS_RESET",
    "07": "IMAGE_SIZE_REQ",
    "08": "PKT_RCPT_REQ",
    "10": "RESPONSE",
    "11": "PKT_RCPT_NOTIF",
}

DFU_STATUS = {
    "01": "SUCCESS",
    "02": "INVALID_STATE",
    "03": "NOT_SUPPORTED",
    "04": "DATA_SIZE",
    "05": "CRC_ERROR",
    "06": "OPER_FAILED",
}

CCCD = "00002902-0000-1000-8000-00805f9b34fb"
SERVICE = "00001530-1212-efde-1523-785feabcd123"
DFU_CONTROL_POINT = "00001531-1212-efde-1523-785feabcd123"
DFU_PACKET = "00001532-1212-efde-1523-785feabcd123"
DFU_VERSION = "00001534-1212-efde-1523-785feabcd123"


def convert_uint32_to_array(value):
    """Convert a number into an array of 4 bytes (LSB).
    This has been modified to prepend 8 zero bytes per the new DFU spec."""
    return [
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        (value >> 0 & 0xFF),
        (value >> 8 & 0xFF),
        (value >> 16 & 0xFF),
        (value >> 24 & 0xFF),
    ]


def convert_uint16_to_array(value):
    return [(value >> 0 & 0xFF), (value >> 8 & 0xFF)]


def convert_array_to_hex_string(arr):
    hex_str = ""
    for val in arr:
        if val > 255:
            raise Exception(
                "Value is greater than it is possible to represent with one byte"
            )
        hex_str += "%02x" % val

    return binascii.a2b_hex(hex_str)


def get_dfu_control_point_handle(ble_connection, uuid):
    try:
        dfu_service = ble_connection.getServiceByUUID(
            uuid
        )
        for characteristic in dfu_service.getCharacteristics():
            if characteristic.uuid == DFU_CONTROL_POINT:
                return characteristic.getHandle() + 1, characteristic.getHandle()
        else:
            LOG.error("Error: DFU Control Point characteristic was not found on this device!")
            return 0, 0
    except Exception:
        LOG.exception("Error: DFU service not found on this device!")
        return 0, 0


def get_handle(ble_connection, uuid):
    LOG.info("getHandle ", uuid)

    for descriptor in ble_connection.getDescriptors():
        if descriptor.uuid == uuid:
            return descriptor.handle
    return False


class MyDelegate(bluepy.btle.DefaultDelegate):
    instance = None

    def __init__(self, instance):
        super().__init__()
        self.instance = instance

    def handleNotification(self, c_handle, data):
        if self.instance is not None:
            method = getattr(self.instance, "set_notification")
            method(data)


class BleDfuServer:

    def __init__(self, target_mac, zip_file):
        self.target_mac = target_mac
        self.zip_file = zip_file

        with zipfile.ZipFile(zip_file, "r") as input_file:
            self.bin_array = array("B", input_file.read('application.bin'))
            self.hex_size = len(self.bin_array)
            self.dat_file = array("B", input_file.read("application.dat"))

        self.delegate = MyDelegate(self)
        self.ble_connection = None
        self.connected = False
        self.notification = None

        self.ctrlpt_handle = 0x10
        self.ctrlpt_cccd_handle = 0x11
        self.data_handle = 0x0E
        self.reset_handle = 0x13
        self.ctrlpt_cccd_handle_buttonless = 0x14

        # TODO Check this parameter to speed up the rprocedure
        self.pkt_receipt_interval = 10  # DEFAULT=10
        self.pkt_payload_size = 20  # DEFAULT=20
        self.connected = False

    def __enter__(self):
        self.ble_connection = bluepy.btle.Peripheral(self.target_mac, "random")
        self.ble_connection.setDelegate(self.delegate)
        LOG.info("connected!")
        self.ctrlpt_cccd_handle_buttonless, self.reset_handle = get_dfu_control_point_handle(
            self.ble_connection, SERVICE
        )
        LOG.info("END scan_and_connect")
        self.connected = True
        return self

    def __exit__(self, *args):
        self.ble_connection.disconnect()

    def set_notification(self, notify):
        self.notification = notify

    def send_image(self):
        """Send the binary firmware image to peripheral device."""
        LOG.info("dfu_send_image")

        if not self._check_dfu_mode():
            self.switch_in_dfu_mode()

        LOG.info("Enable Notifications in DFU mode")
        self._dfu_enable_cccd(True)

        # TODO Handle softdevice and bootloader upgrade
        # Send 'START DFU' + Application Command
        self._dfu_state_set(0x0104)

        # Transmit binary image size
        hex_size_array_lsb = convert_uint32_to_array(self.hex_size)
        LOG.info(f"bin array length = {self.hex_size}")
        # self.ble_conn.setDelegate(self.delegate)
        LOG.info(hex_size_array_lsb)

        self._dfu_data_send_req(hex_size_array_lsb)
        LOG.info("Sending hex file size")
        # self.ble_conn.setDelegate(self.delegate)

        LOG.info("waiting for notification...")
        received = self.ble_connection.waitForNotifications(10.0)

        if not received:
            raise Exception("Notification not received. Device didn't reply")
        else:
            dfu_status = self._dfu_parse_notify(self.notification)
            if dfu_status != "OK":
                raise Exception("bad notification status")

        # Send 'INIT DFU' Command
        self._dfu_state_set(0x0200)

        # Transmit the Init image (DAT).
        self._dfu_send_init()

        # Send 'INIT DFU' + Complete Command
        self._dfu_state_set(0x0201)

        # Send packet receipt notification interval (currently 10)
        self._dfu_pkt_rcpt_notif_req()

        # Send 'RECEIVE FIRMWARE IMAGE' command to set DFU in firmware receive state.
        self._dfu_state_set_byte(RECEIVE_FIRMWARE_IMAGE)

        # Send bin_array contents as as series of packets (burst mode).
        # Each segment is pkt_payload_size bytes long.
        # For every pkt_receipt_interval sends, wait for notification.

        segment_count = 1
        for i in range(0, self.hex_size, self.pkt_payload_size):
            segment = self.bin_array[i: i + self.pkt_payload_size]
            self._dfu_data_send_cmd(segment)

            if (segment_count % self.pkt_receipt_interval) == 0:
                self.ble_connection.waitForNotifications(30.0)
                if self.notification is None:
                    raise Exception("no notification received")
                dfu_status = self._dfu_parse_notify(self.notification)
                if dfu_status is None or dfu_status != "OK":
                    raise Exception("bad notification status")
            segment_count += 1

        # Send Validate Command
        self._dfu_state_set_byte(VALIDATE_FIRMWARE_IMAGE)

        # Wait a bit for copy on the peer to be finished
        time.sleep(1)

        # Send Activate and Reset Command
        self._dfu_state_set_byte(ACTIVATE_FIRMWARE_AND_RESET)

    def switch_in_dfu_mode(self):
        """Enable CCD to switch in DFU mode"""
        LOG.info("switch_in_dfu_mode")

        if not self._dfu_enable_cccd(False):
            return False
        time.sleep(0.5)

        # TODO handle softdevice and bootloader upgrade
        # Reset the board in DFU mode. After reset the board will be disconnected
        LOG.info(str("char-write-req 0x%02x 0104" % self.reset_handle))
        # char-write-req 0x0013 0104
        self.ble_connection.writeCharacteristic(
            self.reset_handle, str(chr(0x01)) + str(chr(0x04)), True
        )
        self.ble_connection.disconnect()
        time.sleep(1)

        LOG.info("END switch_in_dfu_mode")
        # Reconnect the board.
        LOG.info("Connected ")

    def _dfu_parse_notify(self, notify):
        """Parse notification status results"""
        if len(notify) < 3:
            LOG.info("notify data length error")
            return "FAIL"
        dfu_oper = hex(notify[0])
        dfu_oper = str(dfu_oper[2:])
        oper_str = DFU_OPERATION[dfu_oper]
        LOG.info(f"_dfu_parse_notify: {notify} dfu_oper: {dfu_oper}")
        if oper_str == "RESPONSE":
            proc = hex(notify[1])
            proc = str(proc[2:])
            if len(proc) == 1:
                dfu_process = "0" + proc
            else:
                dfu_process = proc

            stat = hex(notify[2])
            stat = str(stat[2:])
            if len(stat) == 1:
                dfu_status = "0" + stat
            else:
                dfu_status = stat

            process_str = DFU_PROCEDURE[dfu_process]
            status_str = DFU_STATUS[dfu_status]

            LOG.info(f"oper: {oper_str}, proc: {process_str}, status: {status_str}")

            if oper_str == "RESPONSE" and status_str == "SUCCESS":
                return "OK"
            else:
                LOG.error("ERROR: [_dfu_parse_notify]")
                return "FAIL"

        if oper_str == "PKT_RCPT_NOTIF":
            byte1 = notify[4]
            byte2 = notify[3]
            byte3 = notify[2]
            byte4 = notify[1]

            receipt = 0
            receipt = receipt + (byte1 << 24)
            receipt = receipt + (byte2 << 16)
            receipt = receipt + (byte3 << 8)
            receipt = receipt + (byte4 << 0)

            LOG.info("PKT_RCPT: {0:8}".format(receipt) + " of " + str(self.hex_size))
            return "OK"

    def _dfu_state_set(self, opcode):
        """ Send two bytes: command + option"""
        try:
            opcode = hex(opcode)
        except TypeError:
            pass
        LOG.info('char-write-req 0x%04x %s' % (self.ctrlpt_handle, opcode))

        if len(opcode) == 5:
            send_opcode = opcode[0:2] + "0" + opcode[2:]
        else:
            send_opcode = opcode

        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle,
            bytes(chr(int(send_opcode[2:4], 16)) + chr(int(send_opcode[4:6], 16)), 'utf-8'),
            True
        )

    def _dfu_state_set_byte(self, opcode):
        """Send one byte: command"""
        try:
            opcode = hex(opcode)
        except TypeError:
            pass
        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle, bytes(chr(int(opcode[2:4], 16)), 'utf-8'), True
        )

    def _dfu_pkt_rcpt_notif_req(self):
        """Send 3 bytes: PKT_RCPT_NOTIF_REQ with interval of 10 (0x0a)"""
        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle, bytes(chr(0x08) + chr(0x0a) + chr(0x00), 'utf-8'), True
        )

    def _dfu_data_send_req(self, data_arr):
        """Send an array of bytes: request mode"""
        self.ble_connection.writeCharacteristic(
            self.data_handle, convert_array_to_hex_string(data_arr), True
        )

    def _dfu_data_send_cmd(self, data_arr):
        """Send an array of bytes: command mode"""
        self.ble_connection.writeCharacteristic(
            self.data_handle, convert_array_to_hex_string(data_arr), False
        )

    def _dfu_enable_cccd(self, dfu_mode):
        """Enable DFU Control Point CCCD (Notifications)"""
        handle = self.ctrlpt_cccd_handle
        if not dfu_mode:
            handle = self.ctrlpt_cccd_handle_buttonless

        LOG.info("_dfu_enable_cccd")
        cccd_enable_value_array_lsb = convert_uint16_to_array(0x0001)
        cccd_enable_value_hex_string = convert_array_to_hex_string(
            cccd_enable_value_array_lsb
        )
        command = str(
            "char-write-req 0x%04x %s" % (handle, cccd_enable_value_hex_string)
        )
        LOG.info(command)

        descriptors = self.ble_connection.getDescriptors()
        handle = None
        for descriptor in descriptors:
            if descriptor.uuid == DFU_CONTROL_POINT:
                handle = descriptor.handle + 1

        try:
            self.ble_connection.writeCharacteristic(
                handle, cccd_enable_value_hex_string, True
            )
            return True
        except Exception:
            return False

    def _dfu_send_init(self):
        """Send the Init info (*.dat file contents) to peripheral device."""
        LOG.info("dfu_send_info")
        self._dfu_data_send_req(self.dat_file)

    def _dfu_check_mode(self):
        self._dfu_get_handles()

        LOG.info("_dfu_check_mode")
        # look for DFU switch characteristic

        reset_handle = get_handle(self.ble_connection, DFU_CONTROL_POINT)

        LOG.info("resetHandle ", reset_handle)

        self.ctrlpt_cccd_handle = None

        if not reset_handle:
            self.ctrlpt_handle = get_handle(self.ble_connection, DFU_CONTROL_POINT)
            if not self.ctrlpt_handle:
                LOG.info("Not in DFU, nor has the toggle characteristic, aborting..")
                return False

        if reset_handle or self.ctrlpt_handle:
            if reset_handle:
                LOG.info("Switching device into DFU mode")
                LOG.info("char-write-cmd 0x%02s %02x" % (reset_handle, 1))
                self.ble_connection.writeCharacteristic(reset_handle, str(chr(0x01)), True)
                time.sleep(0.2)

                LOG.info("Node is being restarted")
                self.ble_connection.disconnect()

                # wait for restart
                time.sleep(5)
                LOG.info("Reconnecting...")

                # reinitialize
                self.__init__(self.target_mac, self.zip_file)

                if not self.connected:
                    return False
                return self._dfu_check_mode()
            else:
                LOG.info("Node is in DFU mode")
            return True
        else:
            return False

    def _dfu_get_handles(self):
        LOG.info("_dfu_get_handles")
        self.ctrlpt_cccd_handle = "10"
        self.data_handle = "0e"

        ctrlpt_cccd_handle = get_handle(
            self.ble_connection, CCCD
        )
        data_handle = get_handle(self.ble_connection, DFU_PACKET)
        LOG.info("ctrlpt_cccd_handle ", ctrlpt_cccd_handle)
        LOG.info("data_handle ", data_handle)

        if ctrlpt_cccd_handle:
            self.ctrlpt_cccd_handle = ctrlpt_cccd_handle
        if data_handle:
            self.data_handle = data_handle

    def _check_dfu_mode(self):
        """Return True if is already in DFU mode"""
        LOG.info("Checking DFU State...")
        res = False
        dfu_version_characteristic = self.ble_connection.getCharacteristics(
            uuid=DFU_VERSION
        )[0]
        if dfu_version_characteristic is not None:
            version = dfu_version_characteristic.read()
            if int(version[0]) == 8:
                res = True
                LOG.info("Board already in DFU mode")
            elif int(version[0]) == 1:
                LOG.info("Board needs to switch in DFU mode")
        return res


def main():
    logging.config.dictConfig(LOG_CONFIG)
    LOG.info("DFU Server start")
    sys.dont_write_bytecode = True

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a", "--address", type=str, required=True, help="DFU target MAC address", action="store"
    )

    parser.add_argument(
        "-z", "--zipfile", type=str, help="Firmware as zip archive", action="store"
    )
    args = parser.parse_args()

    with BleDfuServer(args.address.upper(), args.zipfile) as ble_dfu:
        ble_dfu.send_image()

    LOG.debug("DFU Server done")


if __name__ == "__main__":
    main()
