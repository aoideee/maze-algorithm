import socket
import threading
import time
from typing import Optional

from config import (
    DRONE_IP,
    DRONE_CMD_PORT,
    HOST_CMD_PORT,
    HOST_STATE_PORT,
)


class DroneController:
    """
    Manages UDP communication with the Tello drone.
    Command responses and telemetry are handled in dedicated background threads.
    """

    def __init__(self) -> None:
        self._addr = (DRONE_IP, DRONE_CMD_PORT)
        self._last_response: Optional[str] = None
        self._telemetry: dict[str, str] = {}
        self._active = True

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind(("", HOST_CMD_PORT))
        self._cmd_sock.settimeout(5)

        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.bind(("", HOST_STATE_PORT))
        self._state_sock.settimeout(1)

        threading.Thread(target=self._poll_responses, daemon=True).start()
        threading.Thread(target=self._poll_state, daemon=True).start()

    def _poll_responses(self) -> None:
        while self._active:
            try:
                data, _ = self._cmd_sock.recvfrom(1024)
                self._last_response = data.decode("utf-8").strip()
            except socket.timeout:
                continue
            except OSError:
                break

    def _poll_state(self) -> None:
        while self._active:
            try:
                data, _ = self._state_sock.recvfrom(2048)
                self._telemetry = self._decode_state(data.decode("utf-8").strip())
            except socket.timeout:
                continue
            except OSError:
                break

    @staticmethod
    def _decode_state(raw: str) -> dict[str, str]:
        result = {}
        for entry in raw.split(";"):
            if ":" in entry:
                k, v = entry.split(":", 1)
                result[k] = v
        return result

    def send(self, cmd: str, timeout: float = 7.0) -> Optional[str]:
        """
        Dispatch a command to the drone and block until a response arrives
        or the timeout elapses. Returns the response string, or None on timeout.
        """
        self._last_response = None
        self._cmd_sock.sendto(cmd.encode("utf-8"), self._addr)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._last_response is not None:
                return self._last_response
            time.sleep(0.05)

        return None

    # --- High-level flight commands ---

    def enter_sdk_mode(self) -> Optional[str]:
        return self.send("command")

    def stream_on(self) -> Optional[str]:
        return self.send("streamon")

    def stream_off(self) -> Optional[str]:
        return self.send("streamoff")

    def takeoff(self) -> Optional[str]:
        return self.send("takeoff", timeout=10)

    def land(self) -> Optional[str]:
        return self.send("land", timeout=10)

    def move_up(self, cm: int) -> Optional[str]:
        return self.send(f"up {cm}")

    def move_forward(self, cm: int) -> Optional[str]:
        return self.send(f"forward {cm}")

    def move_back(self, cm: int) -> Optional[str]:
        return self.send(f"back {cm}")

    def rotate_cw(self, deg: int) -> Optional[str]:
        return self.send(f"cw {deg}")

    def rotate_ccw(self, deg: int) -> Optional[str]:
        return self.send(f"ccw {deg}")

    def flip(self, direction: str) -> Optional[str]:
        return self.send(f"flip {direction}", timeout=10)

    def battery_level(self) -> Optional[str]:
        return self.send("battery?")

    def keepalive(self) -> Optional[str]:
        """
        Prevents auto-land by sending a lightweight read command.
        Tello will land on its own after ~15 s of silence.
        """
        return self.battery_level()

    def shutdown(self) -> None:
        self._active = False
        for sock in (self._cmd_sock, self._state_sock):
            try:
                sock.close()
            except OSError:
                pass
