
# --------------------------------------------------------------------------- #
# import the various server implementations
# --------------------------------------------------------------------------- #
from pymodbus.server.sync import StartTcpServer, ModbusTcpServer

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from pymodbus.transaction import ModbusRtuFramer, ModbusBinaryFramer

import rospy
from sensor_msgs.msg import JointState

import signal
import sys
import threading
import time
import os

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.DEBUG)

thread_modbus_server = None
thread_ros_joint_state_publisher = None

class ReusableModbusTcpServer(ModbusTcpServer):
    def __init__(self, context, framer=None, identity=None,
                 address=None, handler=None, **kwargs):
        self.allow_reuse_address = True
        ModbusTcpServer.__init__(self, context, framer, identity, address, handler, **kwargs)

class RosPublisher:
    def __init__(self):
        self.opening = False

    def joint_state_publisher(self):
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        rate = rospy.Rate(10) # 10hz
        state = JointState()
        state.name=['shaft_to_bar_link']
        state.position= [0.0]

        pos_end = 1.42
        pos = 0.0

        while not rospy.is_shutdown():
            # print("[{}] [{}] [{}]".format(os.getpid(), hex(id(self)), self.opening))

            if self.opening == True:  
                if pos < pos_end:
                    pos += 0.1

                    state.position = [pos]
                    state.header.stamp = rospy.Time.now()
                    pub.publish(state)
            else:
                if pos > 0.0:
                    pos -= 0.1

                    state.position = [pos]
                    state.header.stamp = rospy.Time.now()
                    pub.publish(state)
            
            rate.sleep()

class CustomDataBlock(ModbusSparseDataBlock):

    def __init__(self, values, rosPublisher):
        self.ros_publisher = rosPublisher
        super(CustomDataBlock, self).__init__(values)

    def setValues(self, address, value):
        
        if address == 1: #coil 1
            self.ros_publisher.opening = value[0]
        
        # print("[{}] [{}] address {}: value: {}".format(
        #     os.getpid(), 
        #     hex(id(self.ros_publisher)), 
        #     address,
        #     value)
        # )

def run_server(publisher):
    
    block = CustomDataBlock([0]*100,publisher)
    store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
    context = ModbusServerContext(slaves=store, single=True)

    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
    identity.ProductName = 'Pymodbus Server'
    identity.ModelName = 'Pymodbus Server'
    identity.MajorMinorRevision = '2.3.0'

    print("Modbus server started")
    StartTcpServer(context, 
        identity=identity, 
        address=("0.0.0.0", 5022),
        allow_reuse_address=True)

def signal_handler(sig, frame):
    print("\n\nSIGINT caught. Terminating..\n\n")
    os._exit(1)
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    publisher = RosPublisher()

    thread_modbus_server = threading.Thread(target=run_server, args=(publisher,))
    thread_modbus_server.setDaemon(True)
    thread_modbus_server.start()

    # thread_ros_joint_state_publisher = threading.Thread(target=joint_state_publisher)
    # thread_ros_joint_state_publisher.setDaemon(True)
    # thread_ros_joint_state_publisher.start()

    publisher.joint_state_publisher()
