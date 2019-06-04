# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for
# full license information.

import random
import time
import sys
import json
import iothub_client
# pylint: disable=E0611
from iothub_client import IoTHubModuleClient, IoTHubClientError, IoTHubTransportProvider, DeviceMethodReturnValue
from iothub_client import IoTHubMessage, IoTHubMessageDispositionResult, IoTHubError
import components

# messageTimeout - the maximum time in milliseconds until a message times out.
# The timeout period starts at IoTHubModuleClient.send_event_async.
# By default, messages do not expire.
MESSAGE_TIMEOUT = 10000

# Choose HTTP, AMQP or MQTT as transport protocol.  Currently only MQTT is supported.
PROTOCOL = IoTHubTransportProvider.MQTT

lidar = components.Lidar()
servo = components.Servo(components.PWM_PIN)


def module_twin_callback(update_state, payload, user_context):
    global servo
    print ( "\nTwin callback called with:\nupdateStatus = %s\npayload = %s\ncontext = %s" % (update_state, payload, user_context) )
    data = json.loads(payload)
    pitch = data.get('desired', {}).get('servo_angle')
    if pitch:
        servo.to_angle(pitch)


def method_callback(method_name, payload, user_context):
    global lidar, servo
    print('received method call:')
    print('\tmethod name:', method_name)
    print('\tpayload:', str(payload))
    retval = DeviceMethodReturnValue()
    retval.status = 200
    msg = {'type': 'lidar', 'servo_angle': servo.angle, 'distance': lidar.measure()}
    retval.response = json.dumps(msg)
    return retval
 

class HubManager(object):

    def __init__(self, protocol=IoTHubTransportProvider.MQTT):
        self.client_protocol = protocol
        self.client = IoTHubModuleClient()
        self.client.create_from_environment(protocol)

        # set the time until a message times out
        self.client.set_option("messageTimeout", MESSAGE_TIMEOUT)
        self.client.set_module_twin_callback(module_twin_callback, self)
        self.client.set_module_method_callback(method_callback, 0)


def main(protocol):
    try:
        print ( "\nPython %s\n" % sys.version )
        print ( "IoT Hub Client for Python" )

        servo.to_angle(0)
        print('servo angle is {:.1f} deg, lidar reports distance {} cm'.format(servo.angle, lidar.measure()))

        hub_manager = HubManager(protocol)

        print ( "Starting the IoT Hub Python sample using protocol %s..." % hub_manager.client_protocol )
        print ( "The sample is now waiting for messages and will indefinitely.  Press Ctrl-C to exit. ")

        while True:
            time.sleep(1)

    except IoTHubError as iothub_error:
        print ( "Unexpected error %s from IoTHub" % iothub_error )
        return
    except KeyboardInterrupt:
        print ( "IoTHubModuleClient sample stopped" )

if __name__ == '__main__':
    main(PROTOCOL)