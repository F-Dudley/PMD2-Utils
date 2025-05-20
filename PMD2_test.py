import time
from PMD_Utils.PowerMonitor import PowerMonitor

if __name__ == "__main__":

    test_period = 10  # seconds
    test_samples = 100

    with PowerMonitor() as pm:
        time.sleep(test_period)
