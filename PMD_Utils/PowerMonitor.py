import os
import time
from serial import Serial
import serial.tools.list_ports
from threading import Thread, Lock

import numpy as np
import pandas as pd

from .Types import *


FLUSH_TYPES = ["numpy", "csv"]


class PowerMonitor:

    def __init__(
        self,
        flush_dir: str = "./pmd_data",
        flush_name: str = f"energy_data_{time.strftime('%Y%m%d_%H%M%S')}",
        flush_type: str = "numpy",
        concat_flush_type: str = "csv",
        flush_interval: int = 10,
        n_samples: int = 100,
        poll_interval: int = 10,
        baudrate: int = 115200,
        port: str = None,
        hwid: str = None,
        timeout: int = None,
    ):
        assert flush_type in FLUSH_TYPES, f"Flush type must be one of {FLUSH_TYPES}"
        assert (
            concat_flush_type in FLUSH_TYPES
        ), f"Final flush type must be one of {FLUSH_TYPES}"
        assert n_samples > 0, "Number of samples must be greater than 0"

        self.flush_name = flush_name
        self.flush_dir = os.path.join(os.path.abspath(flush_dir), flush_name)
        self.flush_type = flush_type
        self.concat_flush_type = concat_flush_type
        self.flush_interval = flush_interval
        self._curr_flush_time = float("inf")
        os.makedirs(flush_dir, exist_ok=True)

        self.n_samples = n_samples
        self.poll_interval = poll_interval
        self.timeout = timeout

        self.device_port, self.device_desc, self.device_hwid = self._get_best_device(
            port, hwid
        )
        self.baudrate = baudrate
        print(
            f"Using device: {self.device_port} - {self.device_desc} - {self.device_hwid}"
        )

        # Serial Specifics
        self._serial: Serial = None
        self._lock = Lock()
        self._run_collection = False
        self._collection_thread = None

        self._flush_buffer = []
        self._volt_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))
        self._curr_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))
        self._power_buffer = np.zeros((n_samples, SENSOR_POWER_NUM))

    def __del__(self):

        if hasattr(self, "_serial") and self._serial.is_open:
            print("Closing Serial Port")
            self._serial.dtr = False
            self._serial.close()

        if hasattr(self, "_collection_thread") and self._collection_thread.is_alive():
            print("Stopping Collection Thread")
            self._run_collection = False
            self._collection_thread.join()

    def __enter__(self):
        self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        with self._lock:
            if self._serial != None & self._serial.is_open:
                print("Serial Port alread Open")
                return

        serial = Serial(
            port=self.device_port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            rtscts=False,
            dsrtr=False,
        )
        serial.dtr = True

        if self._verify_serial(serial):
            serial.read_all()
            serial.reset_input_buffer()
            serial.reset_output_buffer()

            self._curr_flush_time = time.time() + self.flush_interval

            self._serial = serial
            self._run_collection = True

            self._collection_thread = Thread(
                target=self._perform_collection,
                args=(self.dump_file, self.n_samples, self.poll_interval),
                daemon=True,
            )
            self._collection_thread.start()
            print("Started Power Monitor Collection")

    def stop(self):

        self._run_collection = False
        self._collection_thread.join()

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._serial.flush()

        self._serial.dtr = False
        self._serial.close()

        self.flush_data()
        self._concat_flushed_data()

    def flush_data(self):
        with self._lock:

            flush_amount = len(os.listdir(self.flush_dir))

            match (self.flush_type.lower()):
                case "numpy":
                    np.save(
                        os.path.join(
                            self.flush_dir, f"frag{flush_amount}_{self.flush_name}.npy"
                        ),
                        np.array(self._flush_buffer),
                    )
                case "csv":
                    df = pd.DataFrame(self._flush_buffer)
                    df.to_csv(
                        os.path.join(
                            self.flush_dir, f"frag{flush_amount}_{self.flush_name}.csv"
                        ),
                        index=False,
                    )

            self._flush_buffer = []

    def _concat_flushed_data(self):
        with self._lock:
            frag_files = [
                frag_file
                for frag_file in os.listdir(self.flush_dir)
                if frag_file.endswith(f"_{self.flush_name}.csv")
                or frag_file.endswith(f"_{self.flush_name}.npy")
            ]

            if len(frag_files) == 0:
                return

            match (self.flush_type.lower()):
                case "numpy":
                    data = np.concatenate(
                        [
                            np.load(os.path.join(self.flush_dir, frag))
                            for frag in frag_files
                        ]
                    )

                case "csv":
                    data = pd.concat(
                        [
                            pd.read_csv(os.path.join(self.flush_dir, frag))
                            for frag in frag_files
                        ],
                        ignore_index=True,
                    ).to_numpy()

            match (self.concat_flush_type.lower()):
                case "numpy":
                    np.savez(
                        os.path.join(
                            self.flush_dir, f"EnergyData_{self.flush_name}.npy"
                        ),
                        data,
                    )

                case "csv":
                    cols = [
                        "Timestamp",
                    ] + [
                        f"{SENSOR_DEVICES[i]}_{suffix}"
                        for i in range(SENSOR_POWER_NUM)
                        for suffix in ["Voltage", "Current", "Power"]
                    ]

                    data = pd.DataFrame(data, columns=cols)
                    data.to_csv(
                        os.path.join(
                            self.flush_dir, f"collected_{self.flush_name}.csv"
                        ),
                        index=False,
                    )

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

            if time.time() >= self._curr_flush_time:
                self.flush_data()
                self._curr_flush_time = time.time() + self.flush_interval

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

    def _verify_serial(self, serial: Serial) -> bool:
        """
        Verify the device is connected and compatible.
        """
        vendor_buffer: bytes = None
        serial.write(UART_CMD.CMD_READ_VENDOR_DATA.to_bytes())
        while vendor_buffer is None or len(vendor_buffer) != sizeof(VendorDataStruct):
            vendor_buffer += serial.read(sizeof(VendorDataStruct))
        vendor_data = VendorDataStruct.from_buffer_copy(vendor_buffer)
        print(f"Vendor Data: {vendor_data}")
        assert (
            vendor_data.VendorId == VENDOR_ID and vendor_data.ProductId == PRODUCT_ID
        ), "Vendor ID and Product Mismatch | Expected: 0x{VENDOR_ID:02X}, 0x{PRODUCT_ID:02X} | Got: 0x{vendor_data.VendorId:02X}, 0x{vendor_data.ProductId:02X}"

        assert (
            vendor_data.FwVersion != FIRMWARE_VERSION
        ), f"Firmware Version too low | Expected: {FIRMWARE_VERSION} | Got: {vendor_data.FwVersion}"

        return True
