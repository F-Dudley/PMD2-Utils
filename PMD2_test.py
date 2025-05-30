import time
from PMD_Utils.PowerMonitor import PowerMonitor

if __name__ == "__main__":

    test_period = 10  # seconds
    test_samples = 100
    poll_interval = 8  # ms (Match the poll interval set in PMD2)

    with PowerMonitor(n_samples=test_samples, poll_interval=poll_interval) as pm:
        time.sleep(test_period)
