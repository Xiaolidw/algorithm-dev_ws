#!/usr/bin/env python3
"""Reliably configure and activate the standalone Nav2 map server."""

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


class MapServerBootstrap(Node):
    def __init__(self):
        super().__init__('map_server_bootstrap')
        self.get_state_client = self.create_client(
            GetState, '/map_server/get_state')
        self.change_state_client = self.create_client(
            ChangeState, '/map_server/change_state')
        self.pending = False
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            'Map-server bootstrap waiting for lifecycle services')

    def tick(self):
        if self.pending or not self.get_state_client.service_is_ready():
            return
        self.pending = True
        future = self.get_state_client.call_async(GetState.Request())
        future.add_done_callback(self.state_received)

    def state_received(self, future):
        self.pending = False
        try:
            state_id = int(future.result().current_state.id)
        except Exception as error:
            self.get_logger().warning(f'Cannot read map-server state: {error}')
            return
        if state_id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('Map server is ACTIVE; bootstrap complete')
            self.timer.cancel()
            return
        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            self.request_transition(Transition.TRANSITION_CONFIGURE, 'configure')
        elif state_id == State.PRIMARY_STATE_INACTIVE:
            self.request_transition(Transition.TRANSITION_ACTIVATE, 'activate')
        else:
            self.get_logger().warning(
                f'Map server in transient state {state_id}; retrying')

    def request_transition(self, transition_id, label):
        if not self.change_state_client.service_is_ready():
            return
        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        self.pending = True
        future = self.change_state_client.call_async(request)

        def completed(done):
            self.pending = False
            try:
                success = bool(done.result().success)
            except Exception as error:
                self.get_logger().warning(
                    f'Map-server {label} call failed: {error}')
                return
            if success:
                self.get_logger().info(f'Map-server {label} accepted')
            else:
                self.get_logger().warning(
                    f'Map-server {label} rejected; retrying')

        future.add_done_callback(completed)


def main(args=None):
    rclpy.init(args=args)
    node = MapServerBootstrap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
