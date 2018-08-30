import rclpy
from rclpy.node import Node
import OpenOPC


class OpcNode(Node):

    def __init__(self):
        super().__init__('listener')
        
        gateway='172.16.23.121'
        opchost='172.16.23.121'
        opcserv='Matrikon.OPC.Simulation'
        taglist =['Random.Int4','Random.Real4']

        print('Connecting to Gateway Server on: ',gateway)

        opc = OpenOPC.open_client(gateway)
        opc.connect(opcserv,opchost)
        v = opc.read(taglist)
        opc.close()

        for i in range(len(v)):
            (name, val, qual, time) = v[i]
            print('%-15s %-15s %-15s %-15s'%(name,val,qual,time))
        


def main(args=None):
    rclpy.init(args=args)
    print('OPC node starting')

    node = OpcNode()
    print('OPC node launched')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()