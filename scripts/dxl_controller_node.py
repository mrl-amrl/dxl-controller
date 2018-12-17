#!/usr/bin/env python
import rospy
from dxl_controller.srv import *

from pypot.dynamixel import DxlIO
from pypot.dynamixel.io.abstract_io import DxlCommunicationError


class Controller:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000):
        self.dxl_controller = DxlIO(port=port, baudrate=baudrate)

        rospy.Service(
            '/dxl_controller/set_speed',
            SetSpeed,
            self._set_speed_callback,
            buff_size=20,
        )

        rospy.Service(
            '/dxl_controller/set_position',
            SetPosition,
            self._set_position_callback,
            buff_size=20,
        )

        rospy.Service(
            '/dxl_controller/get_position',
            GetPosition,
            self._get_position_callback,
        )

        rospy.Service(
            '/dxl_controller/get_speed',
            GetSpeed,
            self._get_speed_callback,
            buff_size=20,
        )

        rospy.Service(
            '/dxl_controller/ping',
            Ping,
            self._ping_callback,
            buff_size=20,
        )

    def ping(self, id):
        return self.dxl_controller.ping(id)

    def _get_position_callback(self, req):
        output = GetPositionResponse()
        if not self.ping(req.id):
            rospy.logerr('Error on pinging dxl {}'.format(req.id))
            output.status = False
            return output

        try:
            output.position = int(
                self.dxl_controller.get_position([req.id])[0])
        except DxlCommunicationError as err:
            rospy.logerr(err)
            output.status = False
            return output

        output.status = True
        return output

    def _get_speed_callback(self, req):
        output = GetSpeedResponse()
        if not self.ping(req.id):
            rospy.logerr('Error on pinging dxl {}'.format(req.id))
            output.status = False
            return output

        try:
            output.speed = int(
                self.dxl_controller.get_moving_speed([req.id])[0])
        except DxlCommunicationError as err:
            rospy.logerr(err)
            output.status = False
            return output

        output.status = True
        return output

    def _ping_callback(self, req):
        return self.ping(req.id)

    def _set_speed_callback(self, req):
        if not self.ping(req.id):
            rospy.logerr('Error on pinging dxl {}'.format(req.id))
            return False

        try:
            self.dxl_controller.set_moving_speed({req.id: req.speed})
        except DxlCommunicationError as err:
            rospy.logerr(err)
            return False
        return True

    def _set_position_callback(self, req):
        if not self.ping(req.id):
            rospy.logerr('Error on pinging dxl {}'.format(req.id))
            return False

        try:
            self.dxl_controller.set_goal_position({req.id: req.position})
        except DxlCommunicationError as err:
            rospy.logerr(err)
            return False
        return True

    @staticmethod
    def spin():
        rospy.spin()

    def kill(self):
        self.dxl_controller.close()


if __name__ == "__main__":
    rospy.init_node('dxl_controller')

    node_name = rospy.get_name()
    usb_port = rospy.get_param(node_name + '/usb_port', '/dev/ttyUSB0')
    baudrate = int(rospy.get_param(node_name + '/baudrate', '1000000'))

    controller = Controller(
        port=usb_port,
        baudrate=baudrate
    )

    try:
        controller.spin()
    except rospy.ROSException as err:
        rospy.logerr(err)
    finally:
        controller.kill()
