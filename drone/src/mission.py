import time

from config import (
    BATTERY_FLOOR,
    ASCENT_HEIGHT_CM,
    MAZE_ENTRY_CM,
    FLIP_DIR,
    STEP_PAUSE,
)
from tello import DroneController


class FlightPlan:
    """
    Defines the sequence of drone actions to execute once
    the green LED start signal has been confirmed.
    """

    def __init__(self, drone: DroneController) -> None:
        self.drone = drone

    # def preflight_check(self) -> bool:
    #     resp = self.drone.battery_level()
    #     if resp is None:
    #         print("Battery read failed.")
    #         return False
    #     try:
    #         pct = int(resp)
    #     except ValueError:
    #         print(f"Bad battery response: {resp}")
    #         return False
    #     print(f"Battery: {pct}%")
    #     return pct >= BATTERY_FLOOR

    def _step(self, label: str, result) -> None:
        """Print the result of a flight step, then pause."""
        print(f"{label}: {result}")
        time.sleep(STEP_PAUSE)

    def run(self) -> None:
        """
        Execute the full mission sequence:
          1. Take off
          2. Ascend to clear height
          3. Perform acrobatic flip
          4. Fly into the maze
          5. Final approach and land
        """
        print("--- Flight plan starting ---")

        self._step("Takeoff", self.drone.takeoff())
        self._step(f"Ascend {ASCENT_HEIGHT_CM} cm", self.drone.move_up(ASCENT_HEIGHT_CM))
        self._step(f"Flip ({FLIP_DIR})", self.drone.flip(FLIP_DIR))
        self._step(f"Maze entry {MAZE_ENTRY_CM} cm", self.drone.move_forward(MAZE_ENTRY_CM))
        self._step("Landing", self.drone.land())

        print("--- Flight plan complete ---")
