from ctypes import *
from enum import IntEnum

from .Constants import *


class UART_CMD(IntEnum):
    CMD_WELCOME = 0
    CMD_READ_VENDOR_DATA = 1
    CMD_READ_UID = 2
    CMD_READ_DEVICE_DATA = 3
    CMD_READ_SENSOR_VALUES = 4
    CMD_WRITE_CONT_TX = 5
    CMD_READ_CALIBRATION = 6
    CMD_WRITE_CALIBRATION = 7
    CMD_LOAD_CALIBRATION = 8
    CMD_STORE_CALIBRATION = 9
    CMD_RESET = 0xF0
    CMD_BOOTLOADER = 0xF1
    CMD_NVM_CONFIG = 0xF2
    CMD_NOP = 0xFF

    def to_bytes(self, length=1, byteorder="little", *, signed=False):
        return self.value.to_bytes(length, byteorder, signed=signed)


class VendorDataStruct(Structure):
    __pack__ = 1
    __fields__ = [("VendorId", c_uint8), ("ProductId", c_uint8), ("FwVersion", c_uint8)]

    def __str__(self):
        return f"VendorID: {hex(self.VendorId)}, ProductID: {hex(self.ProductId)}, FwVersion: {hex(self.FwVersion)}"


class PowerSensor(Structure):
    __pack__ = 1
    __fields__ = [("Voltage", c_int16), ("Current", c_int32), ("Power", c_int32)]

    def __str__(self):
        return f"Voltage: {self.Voltage}, Current: {self.Current}, Power: {self.Power}"


class SensorStruct(Structure):
    _pack_ = 1
    _fields_ = [
        ("Vdd", c_uint16),
        ("Tchip", c_int16),
        ("PowerReadings", PowerSensor * SENSOR_POWER_NUM),
        ("EpsPower", c_uint16),
        ("PciePower", c_uint16),
        ("MbPower", c_uint16),
        ("TotalPower", c_uint16),
        ("Ocp", c_uint8 * SENSOR_POWER_NUM),
    ]

    def __str__(self):
        vin = ", ".join(str(v) for v in self.Vin)
        ts = ", ".join(str(t) for t in self.Ts)
        power_readings = ", ".join(str(p) for p in self.PowerReadings)
        fans = ", ".join(str(f) for f in self.Fans)
        return f"Vdd:{self.Vref}\nTchip: {self.FanExt}\nPowerReadings: {power_readings}"
