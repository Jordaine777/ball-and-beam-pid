import serial
import matplotlib.pyplot as plt
import time

PORT = "/dev/cu.usbmodem1201"   # replace with yours
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)

times = []
distances = []
target = None
t0 = None

print("Recording... let the car settle, then press Ctrl-C to plot.")

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        # parse "distance:18.42,target:18.00" into a dict
        data = {}
        for pair in line.split(","):
            if ":" in pair:
                key, value = pair.split(":")
                try:
                    data[key] = float(value)
                except ValueError:
                    pass

        if "distance" in data:
            if t0 is None:
                t0 = time.time()
            times.append(time.time() - t0)
            distances.append(data["distance"])
        if "target" in data:
            target = data["target"]

except KeyboardInterrupt:
    print(f"\nStopped. Captured {len(distances)} samples.")
    ser.close()

    plt.plot(times, distances, label="distance")
    if target is not None:
        plt.axhline(target, color="gray", linestyle="--", label="target")
    plt.xlabel("time (s)")
    plt.ylabel("distance")
    plt.legend()
    plt.title("car-and-beam PID response Kp + Kd + ki")
    plt.show()