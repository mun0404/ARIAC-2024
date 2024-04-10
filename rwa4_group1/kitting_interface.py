#!/usr/bin/python3

"""
This script defines a ROS2 node for Kitting Process.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ariac_msgs.srv import MoveAGV
from std_srvs.srv import Trigger
from ariac_msgs.srv import SubmitOrder
from ariac_msgs.msg import AGVStatus, KittingTask, AdvancedLogicalCameraImage, Part
# from group1_msgs.msg import OrderStatus
from ariac_msgs.msg import Order
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
import threading
import traceback
from geometry_msgs.msg import Pose
import PyKDL as kdl
from collections import defaultdict


class CustomOrder:
    """
    Class to represent the order msg used in the node.
    Attributes:
        order_id (str): Unique identifier for the order.
        status (str): Current status of the order.
        priority (int): Priority level of the order.
        time_elapsed (int): Time elapsed since receiving the order [seconds].
        agv_num (int): Number of the AGV assigned to the order.
        tray_id (int): ID of the tray associated with the order.
        tray_locked (bool): Flag indicating whether the tray is locked.
        shipped (bool): Flag indicating whether the order has been shipped.
        submitted (bool): Flag indicating whether the order has been submitted for processing.
        processed (bool): Flag indicating whether the order has been processed.
        type (str): Type of the order (e.g., "pick", "place").
        destination (str): Destination location for the order.
    """

    def __init__(self, order_id, priority):
        """
        Initialize the CustomOrder class.
        Args:
            order_id (str): Unique identifier for the order.
            priority (int): Priority level of the order.
        """
        self.order_id = order_id
        self.status = "pending"  # Initial status
        self.priority = priority
        self.time_elapsed = 0
        self.agv_num = 0
        self.tray_id = 0
        self.tray_locked = False
        self.shipped = False
        self.submitted = False
        self.processed = False
        self.type = None
        self.parts = []
        self.color = None
        self.destination = None
        self.tray_lock_action = False
        self.agv_move_action = False
        self.submit_action = False
        self.left_tray_camera = None
        self.right_tray_camera = None
        self.left_bin_camera = None
        self.right_bin_camera = None


class KittingNode(Node):
    """
    Class to represent a kitting node that interfaces with the warehouse and AGV.
    """

    def __init__(self):
        super().__init__('kitting_node')
        self._logger = self.get_logger()

        try:
            self._logger.info('Kitting node started!')

            # Create multithreaded executor callback groups for managing subscriptions, timers, and services
            self._group = ReentrantCallbackGroup()
            self._group2 = None
            self._group3 = ReentrantCallbackGroup()

            # Create a subscriber to receive orders
            self._order_subscriber = self.create_subscription(
                Order,
                '/ariac/orders',
                self._order_cb,
                QoSProfile(depth=10),
                callback_group=self._group
            )

            # Create a publisher to indicate if orders are submitted
            self._order_submitted_publisher = self.create_publisher(
                Bool,
                '/submitted_orders',
                10,
                callback_group=self._group
            )

            # Create a subscriber to receive the camera images
            self._left_tray_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/tray_camera_2/image',
                self._tray_camera_2_cb,
                qos_profile=rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                 history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1),
                callback_group=self._group
            )

            self._right_tray_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/tray_camera_1/image',
                self._tray_camera_1_cb,
                qos_profile=rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                 history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1),
                callback_group=self._group
            )

            self._left_bin_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/bin_camera_2/image',
                self._bin_camera_2_cb,
                qos_profile=rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                 history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1),
                callback_group=self._group
            )
            self._right_bin_cam_subscriber = self.create_subscription(
                AdvancedLogicalCameraImage,
                '/ariac/sensors/bin_camera_1/image',
                self._bin_camera_1_cb,
                qos_profile=rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                                 history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1),
                callback_group=self._group
            )

            # Timer for periodic tasks
            self._submit_order_status = 0
            self._timer = self.create_timer(
                1, self.process_orders, callback_group=self._group)

            # Initialize variables
            self._orders_queue = []
            self._paused_orders = []
            self._tray = {}
            self._bins = defaultdict(list)
            self._current_order = None
            self._order_lock = threading.Lock()
            self._parts = {'color': {Part.RED: 'Red', Part.GREEN: 'Green', Part.BLUE: 'Blue', Part.ORANGE: 'Orange', Part.PURPLE: 'Purple'},
                           'type': {Part.BATTERY: 'Battery', Part.PUMP: 'Pump', Part.SENSOR: 'Sensor', Part.REGULATOR: 'Regulator'}}

            self.left_bin_camera_flag = False
            self.right_bin_camera_flag = False
            self.left_tray_camera_flag = False
            self.right_tray_camera_flag = False

        except Exception as e:
            self._logger.error(
                f"Failed while initializing Class KittingNode. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _find_transform(self, part_pose_in_camera_frame, camera_pose_in_world_frame):

        # First frame
        camera_orientation = camera_pose_in_world_frame.orientation
        camera_x = camera_pose_in_world_frame.position.x
        camera_y = camera_pose_in_world_frame.position.y
        camera_z = camera_pose_in_world_frame.position.z

        frame_camera_world = kdl.Frame(
            kdl.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            kdl.Vector(camera_x, camera_y, camera_z),
        )

        # Second frame
        part_orientation = part_pose_in_camera_frame.orientation
        part_x = part_pose_in_camera_frame.position.x
        part_y = part_pose_in_camera_frame.position.y
        part_z = part_pose_in_camera_frame.position.z

        frame_part_camera = kdl.Frame(
            kdl.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            kdl.Vector(part_x, part_y, part_z),
        )

        # Multiply the two frames
        frame_part_world = frame_camera_world * frame_part_camera

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = round(frame_part_world.p.x(),6)
        pose.position.y = round(frame_part_world.p.y(),6)
        pose.position.z = round(frame_part_world.p.z(),6)

        q = frame_part_world.M.GetQuaternion()
        pose.orientation.x = round(q[0],3)
        pose.orientation.y = round(q[1],3)
        pose.orientation.z = round(q[2],3)
        pose.orientation.w = round(q[3],3)
        
        return pose


    def _tray_camera_2_cb(self, msg):
        """
        Callback function to handle the incoming left tray camera images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming left tray camera image.
        """

        tray_poses = msg.tray_poses
        camera_pose = msg.sensor_pose
        if not self.left_tray_camera_flag:
            if len(tray_poses) > 0:
                for tray_pose in tray_poses:
                    tray_id = tray_pose.id

                    world_tray_pose = self._find_transform(
                        tray_pose.pose, camera_pose)
                    
                    euler = self.quaternion_to_euler(world_tray_pose.orientation)
                    self._tray[tray_id] = {'position': [world_tray_pose.position.x, world_tray_pose.position.y, world_tray_pose.position.z], 'orientation': [euler[0],euler[1],euler[2] ]}
                    print("Tray cam 2", self._tray)
            self.left_tray_camera_flag = True

    def _tray_camera_1_cb(self, msg):
        """
        Callback function to handle the incoming right tray camera images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming right tray camera image.
        """
        # self._current_order.right_tray_camera = msg
        tray_poses = msg.tray_poses
        camera_pose = msg.sensor_pose
        if not self.right_tray_camera_flag:
            if len(tray_poses) > 0:
                for tray_pose in tray_poses:
                    tray_id = tray_pose.id

                    world_tray_pose = self._find_transform(
                        tray_pose.pose, camera_pose)
                    # self._logger.info(f"Tray pose: {world_tray_pose}")
                    euler =  self.quaternion_to_euler(world_tray_pose.orientation)
                    self._tray[tray_id] = {'position': [world_tray_pose.position.x, world_tray_pose.position.y, world_tray_pose.position.z], 'orientation': [euler[0],euler[1],euler[2] ]}
                    
            self.right_tray_camera_flag = True

    def _bin_camera_2_cb(self, msg):
        """
        Callback function to handle the incoming left bin camera images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming left bin camera image.
        """
        if not self.left_bin_camera_flag:
            bin_poses = msg.part_poses
            for bin_pose in bin_poses:
                bin_part = bin_pose.part
                bin_part_pose = Pose()
                bin_part_pose = bin_pose.pose

                world_bin_pose = self._find_transform(
                    bin_part_pose, msg.sensor_pose)
                euler = self.quaternion_to_euler(world_bin_pose.orientation)
                self._bins[(bin_part.type, bin_part.color)].append({'position': [world_bin_pose.position.x, world_bin_pose.position.y, world_bin_pose.position.z], 'orientation': [euler[0],euler[1],euler[2] ]})

            self.left_bin_camera_flag = True

    def quaternion_to_euler(self, q):
        rpy = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY()
        
        #round off to 3 decimal places
        rpy = [round(rpy[0],1), round(rpy[1],1), round(rpy[2],2)]
        return rpy
    
    def _bin_camera_1_cb(self, msg):
        """
        Callback function to handle the incoming right bin camera images.
        Args:
            msg (ariac_msgs/msg/AdvancedLogicalCameraImage): Incoming right bin camera image.
        """
        if not self.right_bin_camera_flag:
            bin_poses = msg.part_poses
            for bin_pose in bin_poses:
                bin_part = bin_pose.part
                bin_part_pose = Pose()
                bin_part_pose = bin_pose.pose

                world_bin_pose = self._find_transform(
                    bin_part_pose, msg.sensor_pose)
                
                
                
                euler = self.quaternion_to_euler(world_bin_pose.orientation)
                self._bins[(bin_part.type, bin_part.color)].append({'position': [world_bin_pose.position.x, world_bin_pose.position.y, world_bin_pose.position.z], 'orientation': [euler[0],euler[1],euler[2] ]})

            self.right_bin_camera_flag = True

    def process_orders(self):
        """
        Function to simulate the orders for the AGV. This function is called through a timer at a given frequency.
        It iterates through the orders and sets the priority for each order.
        """
        try:
            # Processing current order
            if self._current_order and not self._current_order.processed:

                self._current_order.time_elapsed += 1

            # Check if current order processing time has reached 15 seconds (As defined for Task #5)
            if self._current_order and self._current_order.time_elapsed == 15:
                self._current_order.processed = True

                if not self._current_order.tray_locked and not self._current_order.tray_lock_action:
                    # To get the Unuesd tray

                    self.lock_tray_agv(self._current_order)
                    self._current_order.tray_lock_action = True

                if self._current_order.tray_locked and not self._current_order.shipped and not self._current_order.agv_move_action:
                    self._move_agv(self._current_order)
                    self._current_order.agv_move_action = True

                if self._current_order.shipped and not self._current_order.submit_action:
                    self.submit_order(self._current_order)
                    self._current_order.submit_action

                if self._current_order.submitted:
                   
                    order = self._current_order

                    self._logger.info(f"- {order.order_id}")
                    self._logger.info(" - Kitting Tray:")

                    self._logger.info(f"  - ID:{order.tray_id}")
                    print(self._tray)
                    tray_id = order.tray_id

                    tray_pose = self._tray[tray_id]['position']
                    tray_orientation = self._tray[tray_id]['orientation']
                    self._logger.info(f"  - Position (xyz): {tray_pose}")
                    self._logger.info(
                        f"  - Orientation (rpy): {tray_orientation}")

                    parts = order.parts
                    for part in parts:
                        part_type = self._parts['type'][part.part.type]
                        part_color = self._parts['color'][part.part.color]


                        self._logger.info(f" - {part_color} {part_type}")
                        final_part = None

                        if len(self._bins) > 0:
                            final_part = self._bins[(
                                part.part.type, part.part.color)]

                            for prt in final_part:

                                if (part.part.type, part.part.color) in self._bins.keys():
                                    self._logger.info(
                                        f"   - Position (xyz): {prt['position']}")
                                    self._logger.info(
                                        f"   - Orientation (rpy): {prt['orientation']}")

                                    # pop the part from the bin
                                    self._bins[(part.part.type, part.part.color)].remove(
                                        prt)
                        else:
                            self._logger.info(f" - Part not found in bins.")
                            return
                    self._current_order = None
                    self._orders_queue = self._paused_orders + self._orders_queue
                    self._paused_orders = []

            # Publish status based as 'True' if all orders have been processed
            if not self._orders_queue and not self._current_order:
                _msg = Bool()
                _msg.data = True
                self._order_submitted_publisher.publish(_msg)
            else:
                _msg = Bool()
                _msg.data = False
                self._order_submitted_publisher.publish(_msg)

            # Processing the pending orders
            with self._order_lock:
                # If there are no orders in queue, return
                if not self._orders_queue:
                    return

                # If there is no order in queue or a high priority order is already present in the queue
                if not self._current_order or self._orders_queue[0].priority > self._current_order.priority:
                    if self._current_order:
                        # Move the current order to paused queue
                        self._paused_orders.append(self._current_order)
                        self._logger.info(
                            f"Order #{self._current_order.order_id} is paused.")
                    else:
                        self._logger.info("No current order")

                    # Set current order as the highest priority order from queue
                    self._current_order = self._orders_queue.pop(0)
                    self._logger.info(
                        f"Processing Order #{self._current_order.order_id} with priority '{self._current_order.priority}'.")
                    
                  


        except Exception as e:
            self._logger.error(
                f"Error occurred while processing orders. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _order_cb(self, msg):
        """
        Callback function to processing incoming order msgs from ariac/orders
        Args:
            msg (ariac_msgs/msg/Order): Incoming order msg.
        """
        try:
            # Create a new Order object
            _next_order = CustomOrder(msg.id, msg.priority)
            _next_order.type = msg.type
            _next_order.agv_num = msg.kitting_task.agv_number
            _next_order.tray_id = msg.kitting_task.tray_id
            _next_order.destination = msg.kitting_task.destination
            _next_order.parts = msg.kitting_task.parts
           

            # Add the order to order queue and sort based on priority
            with self._order_lock:
                self._orders_queue.append(_next_order)
                self._orders_queue.sort(
                    key=lambda order: order.priority, reverse=True)

        except Exception as e:
            self._logger.error(
                f"Failed to process incoming order msg from ariac/orders. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def lock_tray_agv(self, order):
        """
        Function to lock the tray of an AGV based on the given order.
        Args:
            order (ariac_msgs/msg/Order): Order msg from the queue.
        """
        try:
            # Create a client to call the service for locking the tray of the specified AGV
            self._lock_agv_tray_client = self.create_client(
                Trigger,
                f'/ariac/agv{order.agv_num}_lock_tray',
                callback_group=self._group
            )

            # Check if the service is ready
            if not self._lock_agv_tray_client.service_is_ready():
                self._logger.warn(
                    'Lock_AGV_Tray Service not available, waiting again...')
                return

            # Trigger service to lock the tray
            self._logger.info(f'Locking tray for AGV #{order.agv_num}')
            request = Trigger.Request()
            future = self._lock_agv_tray_client.call_async(request)

            # Handle the response of locking the tray
            future.add_done_callback(self.lock_agv_tray_response_callback)

        except Exception as e:
            self._logger.error(
                f"Error encountered while locking the AGV tray. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def lock_agv_tray_response_callback(self, future):
        """
        Callback function to handle the response for locking the tray.
        Args:
            future (Future): Future representing the locking of the tray
        """
        try:
            self._logger.info('[LOCK_AGV_TRAY] Response received.')
            if future.result().success:
                self._logger.info('Tray locked!')
                self._current_order.tray_locked = True
            else:
                self._logger.warn('Unable to lock tray!')

        except Exception as e:
            self._logger.error(
                f"Failed to handle LOCK_AGV_TRAY response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def _move_agv(self, order):
        """
        Function to move the AGV to a specified destination based on the given order.
        Args:
            order (ariac_msgs/msg/Order): Order msg from the queue.
        """
        try:
            # Create a client to call the service for moving the AGV
            self._move_agv_client = self.create_client(
                MoveAGV,
                f'/ariac/move_agv{order.agv_num}'
            )

            # Trigger move agv service asynchrononously
            self._logger.debug(
                f"Moving AGV AGV #{order.agv_num} to location '{order.destination}'")  # Enums
            _move_agv_msg = MoveAGV.Request()
            _move_agv_msg.location = order.destination
            future = self._move_agv_client.call_async(_move_agv_msg)

            # Handle the response of moving the AGV
            future.add_done_callback(self.move_agv_response_callback)

        except Exception as e:
            self._logger.error(
                f"Error encountered while moving the AGV. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def move_agv_response_callback(self, future):
        """
        Callback function to handle the response for moving the AGV.
        Args:
            future (Future): Future representing the moving of the AGV.        
        """
        try:
            self._logger.debug('[MOVE_AGV] Response received.')
            if future.result().success:
                # self._logger.info('Shipment successful.')
                self._current_order.shipped = True

                # Create a subscriber for the given AGV
                self._agv_status = self.create_subscription(
                    AGVStatus,
                    f'/ariac/agv{self._current_order.agv_num}_status',
                    self.agv_status_callback,
                    QoSProfile(depth=10),
                    callback_group=self._group3
                )
                self._submit_order_status = 0

        except Exception as e:
            self._logger.error(
                f"Failed to handle MOVE_AGV response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def submit_order(self, order):
        """
        Function to submit the order.
        Args:
            order (CustomOrder): Order to be submitted.
        """
        try:
            # If order is ready to be shipped.
            if order.shipped:
                # Create a client to call the service to submit the order
                self._submit_order_client = self.create_client(
                    SubmitOrder,
                    '/ariac/submit_order',
                    callback_group=self._group
                )

                # # Check if the service is ready
                # if not self._submit_order_client.service_is_ready():
                #     self.logger.warn('Submit_Order Service not available, waiting again...')
                #     return

                # Trigger submit_order service asynchrononously
                self._logger.debug(f'Submitting Order #{order.order_id}...')
                _request = SubmitOrder.Request()
                _request.order_id = order.order_id
                future = self._submit_order_client.call_async(_request)

                # Handle the response for order submission
                future.add_done_callback(self.submit_order_response_callback)

            else:
                self._logger.warn(
                    f'Order #{order.order_id} is not ready for submission')

        except Exception as e:
            self._logger.error(
                f"Error encountering while submitting the order. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def submit_order_response_callback(self, future):
        """
        Callback function to handle the response for submitting the order.
        Args:
            future (Future): Future representing the submission of order.
        """
        try:
            self._logger.debug('[SUBMIT_ORDER] Response received.')
            if future.result().success:
                self._logger.info('Order submitted')
                self._current_order.submitted = True
        except Exception as e:
            self._logger.error(
                f"Failed to handle SUBMIT_ORDER response. Error code: {e}")
            self._logger.error(traceback.format_exc())

    def agv_status_callback(self, msg):
        """
        Callback function to handle the incoming AGV status msgs.
        Args:
            msg (ariac_msgs/msg/AGVStatus): Incoming AGV Status msg.
        """
        try:
            if msg.location == AGVStatus.KITTING and self._submit_order_status == 0:
                self.submit_order(self._current_order)
                self._submit_order_status = 1
        except Exception as e:
            self._logger.error(
                f"Failed to handle incoming AGVStatus msg. Error code: {e}")
            self._logger.error(traceback.format_exc())
