from kone_open_opc_interfaces.srv import *
from kone_open_opc_interfaces.msg import *

import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(GetLiftInfoArray, 'get_lift_info_array')

    req = GetLiftInfoArray.Request()
    req.array = []
    lift_id = LiftId()
    lift_id.gateway_ip = '172.16.23.121'
    lift_id.gateway_port = 7766
    lift_id.opc_serv_ip = 'localhost'
    lift_id.opc_serv_type = 'Matrikon.OPC.Simulation'
    lift_id.site = 'Site'
    lift_id.location = 'Location'
    lift_id.group = 'Group'
    lift_id.lift = 'Lift1'
    req.array.append(lift_id)
    lift_id = LiftId()
    lift_id.gateway_ip = '172.16.23.121'
    lift_id.gateway_port = 7766
    lift_id.opc_serv_ip = 'localhost'
    lift_id.opc_serv_type = 'Matrikon.OPC.Simulation'
    lift_id.site = 'Site'
    lift_id.location = 'Location'
    lift_id.group = 'Group'
    lift_id.lift = 'Lift2'
    req.array.append(lift_id)
    lift_id = LiftId()
    lift_id.gateway_ip = '172.16.23.121'
    lift_id.gateway_port = 7766
    lift_id.opc_serv_ip = 'localhost'
    lift_id.opc_serv_type = 'Matrikon.OPC.Simulation'
    lift_id.site = 'Site'
    lift_id.location = 'Location'
    lift_id.group = 'Group'
    lift_id.lift = 'Lift3'
    req.array.append(lift_id)

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Service call done')
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()