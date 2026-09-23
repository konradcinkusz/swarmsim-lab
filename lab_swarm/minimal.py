"""A swarm written from scratch against swarmsim's system-under-test protocol.

Nothing of swarmsim's own swarm is reused — no DroneController, no MissionSupervisor —
only the types the simulator hands over and takes back: ``Message``, ``Observation``,
``Actuation`` and ``Vector3``. It is the smallest thing that can fly a scenario, and it
exists to show what the protocol asks of a swarm:

* the ground software takes /swarm/mission and /swarm/command payloads (JSON strings in
  swarmsim's rosbridge contract), talks to the drones only by message, and reports the
  mission in the swarm-state shape (``{"mission": {"id", "complete"}}`` is what the
  scenarios read);
* each drone's software reads its own autopilot (a noisy position in its *local* frame,
  armed, mode, battery) and asks for setpoints, a mode and arming — PX4's rules apply:
  no OFFBOARD and no arming without a stream of setpoints.

It flies waypoint missions (one lane per drone) and obeys operator commands. It knows
nothing about formations, batteries or comms loss; ``tests/test_minimal_swarm.py`` holds
it to exactly what it passes and fails in swarmsim's own scenarios.
"""

from __future__ import annotations

import json
from collections.abc import Sequence

from swarm_coordination.scenarios.sim import Actuation, Observation
from swarm_coordination.scenarios.sut import GROUND, DroneInfo, Message
from swarm_coordination.trajectory import Vector3

WARMUP_S = 2.0  # PX4 accepts OFFBOARD only once setpoints are streaming
RETRY_S = 1.0  # how often a refused mode or arming request is repeated
ARRIVED_M = 0.5  # a waypoint is reached within this distance
DONE_PERIOD_S = 1.0  # "done" is repeated: a message can be lost
MODES = {"land": "AUTO.LAND", "rtl": "AUTO.RTL", "hold": "AUTO.LOITER"}


def _number(drone_id: str) -> int:
    return int(drone_id.rsplit("_", 1)[-1])


class MinimalGround:
    def __init__(self, fleet: Sequence[DroneInfo]) -> None:
        self.drones = sorted((d.drone_id for d in fleet), key=_number)
        self.mission_id: str | None = None
        self.mission_drones: list[str] = []
        self.done: set[str] = set()
        self.outbox: list[Message] = []

    def submit_mission(self, payload: str) -> None:
        mission = json.loads(payload)
        if mission["type"] != "waypoint":
            return  # formations are not implemented; the scenarios notice
        waypoints = [Vector3(*w) for w in mission["waypoints"]]
        chosen = self.drones[: mission["drone_count"]]
        self.mission_id, self.mission_drones, self.done = mission["mission_id"], chosen, set()
        for lane, drone in enumerate(chosen):
            shift = Vector3(0.0, lane * mission["spacing_m"], 0.0)
            path = [w + shift for w in waypoints]
            self.outbox.append(
                Message("go", GROUND, drone, {"mission_id": self.mission_id, "path": path})
            )

    def submit_command(self, payload: str) -> None:
        command = json.loads(payload)["command"]
        self.outbox += [Message("command", GROUND, d, command) for d in self.drones]
        self.mission_id = None

    def step(self, now_s: float, inbox: Sequence[Message]) -> list[Message]:
        for message in inbox:
            if message.topic == "done" and message.body == self.mission_id:
                self.done.add(message.sender)
        outgoing, self.outbox = self.outbox, []
        return outgoing

    def report(self) -> dict:
        if self.mission_id is None:
            return {"mission": None}
        complete = set(self.mission_drones) <= self.done
        return {"mission": {"id": self.mission_id, "complete": complete}}


class MinimalDrone:
    def __init__(self, info: DroneInfo) -> None:
        self.info = info
        self.mission_id: str | None = None
        self.path: list[Vector3] = []
        self.phase = "idle"  # idle → warmup → fly → land → done | handed-over
        self.phase_since = 0.0
        self.requested_s = float("-inf")
        self.done_sent_s = float("-inf")
        self.mode_wanted: str | None = None

    def step(
        self, now_s: float, observation: Observation, inbox: Sequence[Message]
    ) -> tuple[Actuation, list[Message]]:
        for message in inbox:
            if message.topic == "go":
                self.mission_id, self.path = message.body["mission_id"], list(message.body["path"])
                self._enter("warmup", now_s)
            elif message.topic == "command":
                self.mode_wanted = MODES[message.body]
                self._enter("handed-over", now_s)

        if observation.local_position is None or self.phase == "idle":
            return Actuation(), []
        here = observation.local_position + self.info.home  # world frame

        if self.phase == "handed-over":  # PX4 flies it now; ask until it says so
            if observation.mode != self.mode_wanted and self._due(now_s):
                return Actuation(request_mode=self.mode_wanted), []
            return Actuation(), []

        if self.phase == "warmup":
            if now_s - self.phase_since >= WARMUP_S:
                self._enter("fly", now_s)
            return Actuation(setpoint_local=here - self.info.home), []

        if self.phase == "fly":
            target = self.path[0]
            if here.distance_to(target) <= ARRIVED_M and observation.armed:
                self.path.pop(0)
                if not self.path:
                    self._enter("land", now_s)
                    return Actuation(request_mode="AUTO.LAND"), []
                target = self.path[0]
            actuation = Actuation(setpoint_local=target - self.info.home)
            if self._due(now_s):
                if observation.mode != "OFFBOARD":
                    actuation = Actuation(actuation.setpoint_local, request_mode="OFFBOARD")
                elif not observation.armed:
                    actuation = Actuation(actuation.setpoint_local, request_arm=True)
            return actuation, []

        if self.phase == "land":
            if observation.armed:
                if observation.mode != "AUTO.LAND" and self._due(now_s):
                    return Actuation(request_mode="AUTO.LAND"), []
                return Actuation(), []
            self._enter("done", now_s)

        # done: landed and disarmed; say so, again and again
        if now_s - self.done_sent_s >= DONE_PERIOD_S:
            self.done_sent_s = now_s
            return Actuation(), [Message("done", self.info.drone_id, GROUND, self.mission_id)]
        return Actuation(), []

    def _enter(self, phase: str, now_s: float) -> None:
        self.phase, self.phase_since, self.requested_s = phase, now_s, float("-inf")

    def _due(self, now_s: float) -> bool:
        if now_s - self.requested_s >= RETRY_S:
            self.requested_s = now_s
            return True
        return False


class MinimalSwarm:
    name = "minimal"

    def ground(self, fleet: Sequence[DroneInfo]) -> MinimalGround:
        return MinimalGround(fleet)

    def drone(self, drone: DroneInfo, fleet: Sequence[DroneInfo]) -> MinimalDrone:
        return MinimalDrone(drone)
