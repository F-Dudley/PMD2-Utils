import os
import time
from serial import Serial
import serial.tools.list_ports
from threading import Thread, Lock

import numpy as np
import pandas as pd

from .Types import *


class PowerMonitor:

    def __init__(
        self,
        dump_dir: str = ".",
        dump_file: str = "energy_data.csv",
        n_samples: int = 100,
        poll_interval: int = 10,
        port: str = None,
        hwid: str = None,
        vid: str = None,
        pid: str = None,
        baudrate: int = 115200,
    ):
        assert os.path.exists(dump_dir), f"Dump directory {dump_dir} does not exist"

        port, desc, hwid = self._get_best_device(port, hwid)
        print(f"Using device: {port} - {desc} - {hwid}")

        serial = Serial(port, baudrate=baudrate, timeout=1, rtscts=False, dsrdtr=False)

        if self._verify_serial(port, hwid, vid, pid):
            self.device_port = port
            self.device_desc = desc
            self.device_hwid = hwid
            self.dump_file: str = os.path.join(dump_dir, dump_file)
            self.n_samples: int = n_samples

            self._serial: Serial = serial
            self._thread = Thread(
                target=self._perform_collection,
                args=(self.dump_file, n_samples, poll_interval),
                daemon=True,
            )
            self._lock = Lock()
            self._run_collection = False

            with self._lock:
                self._flush_buffer = []

                self._volt_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))
                self._curr_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))
                self._power_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))
        else:
            raise RuntimeError(
                f"Warning: Firmware version mismatch. Expected {hex(FIRMWARE_VERSION)}"
            )

    def __del__(self):

        if hasattr(self, "_serial") and self._serial.is_open:
            self._serial.dtr = False
            self._serial.close()

    def __enter__(self):
        self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        self._serial.open()
        self._serial.dtr = True

        self._serial.read_all()

        self._run_collection = True
        self._thread.start()

    def stop(self):

        self._run_collection = False
        self._thread.join()

        self._serial.dtr = False
        self._serial.close()

    def flush_data(
        self, dump_file: str = os.path.join(".", "collected_energy_usage.csv")
    ):
        pass

    def _perform_collection(
        self,
        dump_file: str = os.path.join(".", "collected_energy_usage.csv"),
        n_samples: int = 100,
        poll_interval: int = 10,
    ):
        while self._run_collection:

            buffer = None
            for i in range(n_samples):
                self._serial.write(UART_CMD.CMD_READ_SENSOR_VALUES.to_bytes())

                while buffer == None or len(buffer) != sizeof(SensorStruct):

                    buffer = self._serial.read(sizeof(SensorStruct))
                    if len(buffer) != sizeof(SensorStruct):
                        print(
                            f"Error: Expected {sizeof(SensorStruct)} bytes, got {len(buffer)}"
                        )
                        time.sleep(poll_interval * 0.001)

                sensor_data = SensorStruct.from_buffer_copy(buffer)
                with self._lock:
                    for j in range(SENSOR_POWER_NUM):
                        self._volt_buffer[i, j] = sensor_data.PowerReadings[j].Voltage
                        self._curr_buffer[i, j] = sensor_data.PowerReadings[j].Current
                        self._power_buffer[i, j] = sensor_data.PowerReadings[j].Power

            with self._lock:
                avg_volt = np.average(self._volt_buffer, axis=0)
                avg_curr = np.average(self._curr_buffer, axis=0)
                avg_power = np.average(self._power_buffer, axis=0)

                self._volt_buffer = np.zeros(shape=self._volt_buffer.shape)
                self._curr_buffer = np.zeros(shape=self._curr_buffer.shape)
                self._power_buffer = np.zeros(shape=self._power_buffer.shape)

            print(f"Avg Voltage: {avg_volt}\n\n")
            print(f"Avg Current: {avg_curr}\n\n")
            print(f"Avg Power: {avg_power}\n\n")

            with self._lock:
                entry = {"Timestamp": time.time()}
                for i in range(SENSOR_POWER_NUM):
                    entry[f"{SENSOR_DEVICES[i]}_Voltage"] = avg_volt[i]
                    entry[f"{SENSOR_DEVICES[i]}_Current"] = avg_curr[i]
                    entry[f"{SENSOR_DEVICES[i]}_Power"] = avg_power[i]
                self._flush_buffer.append(entry)

            if len(self._flush_buffer) >= n_samples * 4:
                if os.path.exists(dump_file):
                    df = pd.read_csv(dump_file)

                    df = pd.concat(
                        [
                            df,
                            pd.DataFrame(self._flush_buffer),
                        ],
                        ignore_index=True,
                    )
                else:
                    df = pd.DataFrame(
                        data=self._flush_buffer,
                    )

                df.to_csv(dump_file, index=False)
                self._flush_buffer = []

    def _get_best_device(
        self,
        port=None,
        hwid=None,
    ):
        """
        Get the best available device.
        """
        avail_devices = [
            device
            for device in serial.tools.list_ports.comports()
            if device.hwid.startswith("USB VID:PID=0483:5740")
        ]

        match (len(avail_devices)):
            case 0:
                raise RuntimeError("No PMD device found")
            case 1:
                return avail_devices[0]
            case _:
                print("Multiple PMD devices found:")
                for idx, (dev_port, dev_desc, dev_hwid) in enumerate(
                    sorted(avail_devices)
                ):
                    if dev_port == port:
                        return dev_port, dev_desc, dev_hwid
                    if dev_hwid == hwid:
                        return dev_port, dev_desc, dev_hwid

                    print(f" -- {idx} - {dev_desc} - {dev_hwid}")
                idx = int(input("Select device index: "))

                return avail_devices[idx]

    def _verify_serial(self, serial) -> bool:
        """
        Verify the device is connected and compatible.
        """
        serial.dtr = True

        serial.write(UART_CMD.CMD_READ_VENDOR_DATA.to_bytes())
        vendor_data = VendorDataStruct.from_buffer_copy(serial.read(3))
        print(f"Vendor Data: {vendor_data}")
        assert (
            vendor_data.VendorId == VENDOR_ID and vendor_data.ProductId == PRODUCT_ID
        ), "Vendor ID and Product Mismatch"

        if vendor_data.FwVersion != FIRMWARE_VERSION:
            return False

        serial.read_all()
        serial.close()
        serial.dtr = False

        return True
