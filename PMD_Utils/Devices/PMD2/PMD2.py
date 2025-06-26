import time

from ..Core import PowerDeviceBase


class PMD2(PowerDeviceBase):
    """
    Class representing the PMD2 device.
    Inherits from PMDDevice.
    """

    def __init__(
        self, 
        flush_name: str = f"PMD2_{time.strftime("%Y-%m-%d %H:%M:%S")}", 
        flush_dir: str = "./pmd2_data"
    ):
        super().__init__(flush_name, flush_dir)

        # Set Device Information
        self._info["name"] = "PMD2"

    def __del__(self):
        super().__del__()

    def start(self):
        pass

    def stop(self):
        pass

    def get_info(self):
        pass
