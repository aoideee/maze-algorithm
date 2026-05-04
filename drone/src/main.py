import time

import cv2

from config import STREAM_URL, KEEPALIVE_PERIOD
from mission import FlightPlan
from tello import DroneController
from vision import SignalWatcher


def run() -> None:
    drone = DroneController()
    watcher = SignalWatcher()
    plan = FlightPlan(drone)
    launched = False
    last_ping = time.time()

    try:
        print("Initializing SDK mode...")
        print("Response:", drone.enter_sdk_mode())

        print("Enabling video stream...")
        print("Response:", drone.stream_on())
        time.sleep(2)

        cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            raise RuntimeError("Failed to open drone video stream.")

        # if not plan.preflight_check():
        #     raise RuntimeError("Preflight check failed.")

        print("Watching for green LED signal...")

        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            visible, confirmed, overlay, mask = watcher.analyze(frame)

            if confirmed and not launched:
                print("Signal confirmed — executing flight plan.")
                launched = True
                plan.run()
                break

            if time.time() - last_ping > KEEPALIVE_PERIOD:
                drone.keepalive()
                last_ping = time.time()

            cv2.imshow("Drone Feed", overlay)
            cv2.imshow("Signal Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    finally:
        try:
            drone.stream_off()
        except Exception:
            pass
        drone.shutdown()


if __name__ == "__main__":
    run()
