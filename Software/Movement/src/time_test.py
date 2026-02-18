from skyfield.api import load
import time
import threading



ts = load.timescale()

t0 = ts.utc(2024, 1, 1, 0, 0, 0)
start = time.time()

def now_simulated():
    dt = time.time() - start
    return ts.tt_jd(t0.tt + dt / 86400.0)

def print_simulated_time():
    for _ in range(6):
        print("Simulated time:", now_simulated().utc_iso())
        time.sleep(1)

def print_from_1_to_500():
    for i in range(1, 501):
        print(i)
        time.sleep(0.01)

tread1 = threading.Thread(target=print_simulated_time) 
tread2 = threading.Thread(target=print_from_1_to_500)

tread1.start()
tread2.start()

tread1.join()
tread2.join()