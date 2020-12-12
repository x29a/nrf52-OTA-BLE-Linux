#!/usr/bin/env python
"""
------------------------------------------------------------------------------
 DFU Server for Nordic nRF52 based systems.
 Conforms to nRF52_SDK 11.0 BLE_DFU requirements.
------------------------------------------------------------------------------
"""
import binascii
import logging
import os
import sys
import optparse
import time
import bluepy.btle
import pexpect

from intelhex import IntelHex
from array import array
from unpacker import Unpacker

LOG = logging.getLogger(__name__)


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

    value = binascii.a2b_hex(hex_str)
    value = str(value)
    return value


class MyDelegate(bluepy.btle.DefaultDelegate):
    instance = None

    def __init__(self, instance):
        super().__init__()
        self.instance = instance

    def handleNotification(self, c_handle, data):
        if self.instance is not None:
            method = getattr(self.instance, "setNotification")
            method(data)


class BleDfuServer:
    """ Adjust these handle values to your peripheral device requirements."""

    ctrlpt_handle = 0x10
    ctrlpt_cccd_handle = 0x11
    data_handle = 0x0E
    reset_handle = 0x13
    ctrlpt_cccd_handle_buttonless = 0x14

    # TODO Check this parameter to speed up the rprocedure
    pkt_receipt_interval = 10  # DEFAULT=10
    pkt_payload_size = 20  # DEFAULT=20

    value_written_success_msg = "Characteristic value was written successfully"
    value_written_success_msg_alt = ".* Characteristic value was written successfully"

    notification = None
    delegate = None
    target_mac = None

    def __init__(self, target_mac, hexfile_path, datfile_path):
        super().__init__()
        self.target_mac = target_mac
        self.hexfile_path = hexfile_path
        self.datfile_path = datfile_path
        self.delegate = MyDelegate(self)
        self.ble_connection = None

    def set_notification(self, notify):
        self.notification = notify

    def scan_and_connect(self):
        """Connect to peripheral device."""
        LOG.info("scan_and_connect")
        self.ble_connection = bluepy.btle.Peripheral(self.target_mac, "random")
        self.ble_connection.setDelegate(self.delegate)
        LOG.info("connected!")
        self.ctrlpt_cccd_handle_buttonless, self.reset_handle = get_dfu_control_point_handle(
            self.ble_connection, CCCD
        )
        LOG.info("END scan_and_connect")
        return True

    def _dfu_wait_for_notify(self):
        """Wait for notification to arrive.
        Example format: "Notification handle = 0x0019 value: 10 01 01"""
        while True:
            # print "dfu_wait_for_notify"

            if not self.ble_connection.isalive():
                LOG.info("connection not alive")
                return None

            try:
                index = self.ble_connection.expect(
                    "Notification handle = .*? \r\n", timeout=30
                )

            except pexpect.TIMEOUT:
                #
                # The gatttool does not report link-lost directly.
                # The only way found to detect it is monitoring the prompt '[CON]'
                # and if it goes to '[   ]' this indicates the connection has
                # been broken.
                # In order to get a updated prompt string, issue an empty
                # sendline('').  If it contains the '[   ]' string, then
                # raise an exception. Otherwise, if not a link-lost condition,
                # continue to wait.
                #
                self.ble_connection.sendline("")
                string = self.ble_connection.before
                if "[   ]" in string:
                    LOG.error("Connection lost! ")
                    raise Exception("Connection Lost")
                return None

            if index == 0:
                after = self.ble_connection.after
                LOG.info(f"After: {after}")
                hxstr = after.split()[3:]
                handle = int(float.fromhex(hxstr[0]))
                LOG.info(f"Handle: {handle}")
                return hxstr[2:]

            else:
                LOG.info(f"unexpeced index: {index}")
                return None

    def _dfu_parse_notify(self, notify):
        """Parse notification status results"""
        if len(notify) < 3:
            LOG.info("notify data length error")
            return

        dfu_oper = hex(ord(notify[0]))[2:]
        oper_str = DFU_OPERATION[dfu_oper]
        LOG.info(f"_dfu_parse_notify: {notify} dfu_oper: {dfu_oper}")
        if oper_str == "RESPONSE":
            proc = hex(ord(notify[1]))
            proc = str(proc[2:])
            if len(proc) == 1:
                dfu_process = "0" + proc
            else:
                dfu_process = proc

            stat = hex(ord(notify[2]))
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
            byte1 = ord(notify[4])
            byte2 = ord(notify[3])
            byte3 = ord(notify[2])
            byte4 = ord(notify[1])

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
        except Exception:
            pass
        LOG.info(f"char-write-req 0x{self.ctrlpt_handle}4x {opcode}")
        if len(opcode) == 5:
            send_opcode = opcode[0:2] + "0" + opcode[2:]
        else:
            send_opcode = opcode
        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle,
            str(chr(int(send_opcode[2:4], 16))) + str(chr(int(send_opcode[4:6], 16))),
            True,
        )

    def _dfu_state_set_byte(self, opcode):
        """Send one byte: command"""
        try:
            opcode = hex(opcode)
        except Exception:
            pass
        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle, str(chr(int(opcode[2:4], 16))), True
        )

    def _dfu_pkt_rcpt_notif_req(self):
        """Send 3 bytes: PKT_RCPT_NOTIF_REQ with interval of 10 (0x0a)"""
        opcode = 0x080000
        opcode = opcode + (self.pkt_receipt_interval << 8)  # FIXME: opcode never used anymore

        self.ble_connection.writeCharacteristic(
            self.ctrlpt_handle, str(chr(0x08)) + str(chr(0x0A)) + str(chr(0x00)), True
        )

    def _dfu_data_send_req(self, data_arr):
        """Send an array of bytes: request mode"""
        hex_str = convert_array_to_hex_string(data_arr)
        self.ble_connection.writeCharacteristic(self.data_handle, hex_str, True)

    def _dfu_data_send_cmd(self, data_arr):
        """Send an array of bytes: command mode"""
        hex_str = convert_array_to_hex_string(data_arr)
        self.ble_connection.writeCharacteristic(self.data_handle, hex_str, True)

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
        except Exception:
            return False
        return True

    def _dfu_send_init(self):
        """Send the Init info (*.dat file contents) to peripheral device."""
        LOG.info("dfu_send_info")

        # Open the DAT file and create array of its contents
        bin_array = array("B", open(self.datfile_path, "rb").read())

        # Transmit Init info
        self._dfu_data_send_req(bin_array)

    def input_setup(self):
        """Initialize:
        Hex: read and convert hexfile into bin_array
        Bin: read binfile into bin_array"""
        LOG.info(f"Sending file {self.hexfile_path} to {self.target_mac}")

        if self.hexfile_path is None:
            raise Exception("input invalid")

        name, extent = os.path.splitext(self.hexfile_path)

        if extent == ".bin":
            self.bin_array = array("B", open(self.hexfile_path, "rb").read())
            self.hex_size = len(self.bin_array)
            LOG.info(f"bin array size: {self.hex_size}")
            return

        if extent == ".hex":
            intelhex = IntelHex(self.hexfile_path)
            self.bin_array = intelhex.tobinarray()
            self.hex_size = len(self.bin_array)
            LOG.info(f"bin array size: {self.hex_size}")
            return

        raise Exception("input invalid")

    def _dfu_check_mode(self):
        self._dfu_get_handles()

        LOG.info("_dfu_check_mode")
        # look for DFU switch characteristic

        resetHandle = get_handle(self.ble_connection, DFU_CONTROL_POINT)

        LOG.info("resetHandle ", resetHandle)

        self.ctrlpt_cccd_handle = None

        if not resetHandle:
            self.ctrlpt_handle = get_handle(self.ble_connection, DFU_CONTROL_POINT)
            if not self.ctrlpt_handle:
                LOG.info("Not in DFU, nor has the toggle characteristic, aborting..")
                return False

        if resetHandle or self.ctrlpt_handle:
            if resetHandle:
                LOG.info("Switching device into DFU mode")
                LOG.info("char-write-cmd 0x%02s %02x" % (resetHandle, 1))
                self.ble_connection.writeCharacteristic(resetHandle, str(chr(0x01)), True)
                time.sleep(0.2)

                LOG.info("Node is being restarted")
                self.ble_connection.disconnect()
                time.sleep(0.2)

                # wait for restart
                time.sleep(5)
                LOG.info("Reconnecting...")

                # reinitialize
                self.__init__(self.target_mac, self.hexfile_path, self.datfile_path)

                # reconnect
                connected = self.scan_and_connect()

                LOG.info("connected ")

                if not connected:
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

    def switch_in_dfu_mode(self):
        """Enable CCD to switch in DFU mode"""
        LOG.info("switch_in_dfu_mode")

        if not self._dfu_enable_cccd(False):
            return False
        time.sleep(0.5)

        # TODO handle softdevice and bootloader upgrade
        # Reset the board in DFU mode. After reset the board will be disconnected
        LOG.info(str("char-write-req 0x%02x 0104" % (self.reset_handle)))
        # char-write-req 0x0013 0104
        self.ble_connection.writeCharacteristic(
            self.reset_handle, str(chr(0x01)) + str(chr(0x04)), True
        )
        self.ble_connection.disconnect()
        time.sleep(1)

        LOG.info("END switch_in_dfu_mode")
        # Reconnect the board.
        LOG.info("Connected ", self.scan_and_connect())

    def switch_in_dfu_mode_alt(self):
        LOG.info("switch_in_dfu_mode")
        # scan for characteristics:
        status = self._dfu_check_mode()
        LOG.info("status ", status)
        if not status:
            return False

    def dfu_send_image(self):
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
        hex_size_array_lsb = convert_uint32_to_array(len(self.bin_array))
        LOG.info("bin array length = ", len(self.bin_array))
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

    def _check_dfu_mode(self):
        """Return True if is already in DFU mode"""
        LOG.info("Checking DFU State...")
        res = False
        dfu_version_characteristic = self.ble_connection.getCharacteristics(
            uuid=DFU_VERSION
        )[0]
        if dfu_version_characteristic is not None:
            version = dfu_version_characteristic.read()
            if ord(str(version[0])) == 8:
                res = True
                LOG.info("Board already in DFU mode")
            elif ord(str(version[0])) == 1:
                LOG.info("Board needs to switch in DFU mode")
        return res

    def disconnect(self):
        """Disconnect from peer device if not done already and clean up. """
        self.ble_connection.disconnect()


def get_dfu_control_point_handle(ble_connection, uuid):
    try:
        dfu_service = ble_connection.getServiceByUUID(
            SERVICE
        )
    except Exception:
        LOG.exception("Error: DFU service not found on this device!")
        return 0, 0

    for characteristic in dfu_service.getCharacteristics():
        if characteristic.uuid == DFU_CONTROL_POINT:
            return characteristic.getHandle() + 1, characteristic.getHandle()

    LOG.error("Error: DFU Control Point characteristic was not found on this device!")
    return 0, 0


def get_handle(ble_connection, uuid):
    LOG.info("getHandle ", uuid)

    for descriptor in ble_connection.getDescriptors():
        if descriptor.uuid == uuid:
            return descriptor.handle
    return False


def main():
    LOG.info("DFU Server start")
    sys.dont_write_bytecode = True
    try:
        parser = optparse.OptionParser(
            usage="%prog -f <hex_file> -a <dfu_target_address>\n\nExample:\n\tdfu.py -f application.hex -d application.dat -a cd:e3:4a:47:1c:e4",
            version="0.5",
        )

        parser.add_option(
            "-a",
            "--address",
            action="store",
            dest="address",
            type="string",
            default=None,
            help="DFU target address.",
        )

        parser.add_option(
            "-f",
            "--file",
            action="store",
            dest="hexfile",
            type="string",
            default=None,
            help="hex file to be uploaded.",
        )

        parser.add_option(
            "-d",
            "--dat",
            action="store",
            dest="datfile",
            type="string",
            default=None,
            help="dat file to be uploaded.",
        )

        parser.add_option(
            "-z",
            "--zip",
            action="store",
            dest="zipfile",
            type="string",
            default=None,
            help="zip file to be used.",
        )

        options, args = parser.parse_args()

    except Exception:
        LOG.exception("For help use --help")
        sys.exit(2)

    try:
        if not options.address:
            parser.print_help()
            exit(2)

        hexfile = None
        datfile = None

        if options.zipfile is not None:
            if options.hexfile is not None or options.datfile is not None:
                LOG.error("Conflicting input directives")
                exit(2)

            unpacker = Unpacker()
            try:
                hexfile, datfile = unpacker.unpack_zipfile(options.zipfile)
            except Exception as e:
                LOG.exception(e)

        else:
            if (not options.hexfile) or (not options.datfile):
                parser.print_help()
                exit(2)

            if not os.path.isfile(options.hexfile):
                LOG.error("Error: Hex file doesn't exist")
                exit(2)

            if not os.path.isfile(options.datfile):
                LOG.error("Error: DAT file doesn't exist")
                exit(2)

            hexfile = options.hexfile
            datfile = options.datfile

        # Start of Device Firmware Update processing
        ble_dfu = BleDfuServer(options.address.upper(), hexfile, datfile)

        # Initialize inputs
        ble_dfu.input_setup()

        # Connect to peer device.
        if ble_dfu.scan_and_connect():
            # Transmit the hex image to peer device.
            ble_dfu.dfu_send_image()

            # Wait to receive the disconnect event from peripheral device.
            time.sleep(1)

            # Disconnect from peer device if not done already and clean up.
            ble_dfu.disconnect()

    except Exception as e:
        LOG.exception(e)
    LOG.debug("DFU Server done")


if __name__ == "__main__":
    main()
