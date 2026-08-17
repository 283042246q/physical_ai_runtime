#!/usr/bin/env python3
"""Drive a lifecycle node from unconfigured to active.

Replaces polling `ros2 lifecycle` CLI subprocesses with direct GetState /
ChangeState service calls on a single long-lived rclpy node. This lets the
activator inspect each transition's actual success flag instead of retrying
blindly, and fails fast on terminal states (e.g. finalized) rather than
looping until an overall timeout.
"""

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

# Primary state -> transition required to advance toward active.
_NEXT_TRANSITION = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
}

_EXPECTED_DESTINATION = {
    Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
    Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
}

_TERMINAL_FAILURE_STATES = {State.PRIMARY_STATE_FINALIZED}


class LifecycleActivator(Node):
    def __init__(self, target_node: str, service_call_timeout_s: float):
        super().__init__("lifecycle_activator")
        self._target_node = target_node
        self._service_call_timeout_s = service_call_timeout_s
        self._get_state_client = self.create_client(GetState, f"{target_node}/get_state")
        self._change_state_client = self.create_client(
            ChangeState, f"{target_node}/change_state"
        )

    def wait_for_services(self, deadline: float) -> bool:
        for client, name in (
            (self._get_state_client, "get_state"),
            (self._change_state_client, "change_state"),
        ):
            while not client.wait_for_service(timeout_sec=0.5):
                if time.monotonic() > deadline:
                    self.get_logger().error(
                        f"timed out waiting for {self._target_node}/{name} service"
                    )
                    return False
        return True

    def get_state(self):
        future = self._get_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._service_call_timeout_s
        )
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state.id

    def change_state(self, transition_id: int):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._change_state_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._service_call_timeout_s
        )
        if not future.done() or future.result() is None:
            return False, "change_state call timed out"
        return future.result().success, ""


def run(target_node: str, timeout_s: float, poll_period_s: float) -> int:
    activator = LifecycleActivator(target_node, service_call_timeout_s=5.0)
    deadline = time.monotonic() + timeout_s
    logger = activator.get_logger()

    try:
        if not activator.wait_for_services(deadline):
            return 1

        last_state = None
        while time.monotonic() < deadline:
            state = activator.get_state()
            if state is None:
                logger.warn("get_state call failed or timed out; retrying")
                time.sleep(poll_period_s)
                continue

            last_state = state
            if state == State.PRIMARY_STATE_ACTIVE:
                print(f"[episode_recorder_app] {target_node} lifecycle active")
                return 0

            if state in _TERMINAL_FAILURE_STATES:
                logger.error(f"{target_node} reached terminal state {state}; giving up")
                return 1

            transition = _NEXT_TRANSITION.get(state)
            if transition is None:
                # Node is mid-transition (configuring/activating/error-processing);
                # wait for it to settle instead of forcing another transition.
                time.sleep(poll_period_s)
                continue

            success, error_message = activator.change_state(transition)
            if not success:
                # A rejected transition can be a genuine failure, or a benign
                # race where another driver (e.g. a concurrent activator)
                # already advanced the state between our get_state and
                # change_state calls. Re-check before failing loudly: if the
                # state actually moved, let the next iteration act on it; if
                # it is still stuck where we found it, this is a real error.
                time.sleep(poll_period_s)
                rechecked_state = activator.get_state()
                if rechecked_state == _EXPECTED_DESTINATION[transition]:
                    last_state = rechecked_state
                    continue
                logger.error(
                    f"{target_node} transition {transition} failed: "
                    f"{error_message or 'rejected by node'}"
                )
                return 1

        logger.error(
            f"timed out waiting for {target_node} to become active "
            f"(last_state={last_state})"
        )
        return 1
    finally:
        activator.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "node", help="Fully-qualified lifecycle node name, e.g. /episode_recorder"
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0, help="Overall deadline in seconds"
    )
    parser.add_argument(
        "--poll-period",
        type=float,
        default=0.2,
        help="Seconds between state checks while waiting for a transition",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        return run(args.node, args.timeout, args.poll_period)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
