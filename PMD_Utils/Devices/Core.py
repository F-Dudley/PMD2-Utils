import os
from abc import ABC, abstractmethod
from ctypes import c_uint8
from typing import List, TypedDict, Literal, Optional
from dataclasses import dataclass
from serial import Serial
from time import strftime

fragment_type = Literal["npy", "csv", "json"]
metric_type = Literal["Voltage", "Ampere", "Wattage", "Other"]


class MonitorInfo(TypedDict):
    # Power Monitor Information
    name: str
    desc: Optional[str]
    vendor_id: c_uint8
    product_id: c_uint8
    firmware_version: c_uint8

    timestamp: str

    fragment_type: Optional[fragment_type]


@dataclass
class MonitorFeature:
    name: str
    metric: metric_type
    value: float


class PowerDeviceBase(ABC):
    """
    Abstract Base Class for Elmor-Labs Power Monitoring Devices.
    """

    def __init__(
        self,
        flush_name: str = "Elmor_Device",
        flush_dir: str = "./pmd_data",
        flush_interval: int = 20,
        flush_type: fragment_type = "npy",
        output_dir: str = "./pmd_data",
    ):
        # Default Device Information
        self._info: MonitorInfo = {
            "name": "Unspecified Device",
            "vendor_id": 0,
            "product_id": 0,
            "firmware_version": 0,
            "timestamp": strftime("%Y-%m-%d %H:%M:%S"),
        }

        # Flush Specifics
        self._flush_name = flush_name
        self._flush_path = os.path.join(flush_dir, flush_name)
        self._flush_fragment_type = flush_type
        self._flush_interval = flush_interval

        os.makedirs(flush_dir, exist_ok=True)

        # Serial Specifics
        self._serial: Serial = None

    def __del__(self):
        if hasattr(self, "_serial") and self._serial is not None:
            self._serial.dtr = False
            if self._serial.is_open:
                self._serial.close()

    def __enter__(self):
        self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    @abstractmethod
    def start(self):
        """
        Start Capturing data from the device.
        """
        pass

    @abstractmethod
    def stop(self):
        """
        Stop Capturing data from the device.
        """
        pass

    def info(self) -> MonitorInfo:
        """
        Gets a Dict contaning information about the device.
        """
        return self._info
