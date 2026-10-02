"""Explicit retained ROS protocol, simulation, handler and node owners."""

ROS_GROUPS = {
    "protocol": {
        "sources": [
            "src/chrono_ros/core/ChROSCdr.cpp",
            "src/chrono_ros/core/ChROSSchema.cpp",
            "src/chrono_ros/core/ChROSMessageCodec.cpp",
            "src/chrono_ros/core/ChROSControl.cpp",
            "src/chrono_ros/core/transport/ChROSRingBuffer.cpp",
            "src/chrono_ros/core/transport/ChROSSharedMemory.cpp",
            "src/chrono_ros/core/transport/ChROSChannel.cpp",
        ],
        "headers": [
            "src/chrono_ros/core/ChROSCdr.h",
            "src/chrono_ros/core/ChROSSchema.h",
            "src/chrono_ros/core/ChROSMessageCodec.h",
            "src/chrono_ros/core/ChROSControl.h",
            "src/chrono_ros/core/transport/ChROSRingBuffer.h",
            "src/chrono_ros/core/transport/ChROSSharedMemory.h",
            "src/chrono_ros/core/transport/ChROSChannel.h",
            "src/chrono_ros/core/ChROSFrame.h",
            "src/chrono_ros/core/ChROSQoSSpec.h",
        ],
    },
    "simulation": {
        "sources": [
            "src/chrono_ros/ChROSMessage.cpp",
            "src/chrono_ros/ChROSBridge.cpp",
            "src/chrono_ros/ChROSHandler.cpp",
            "src/chrono_ros/ChROSManager.cpp",
            "src/chrono_ros/ChROSNodeProcess.cpp",
        ],
        "headers": [
            "src/chrono_ros/ChROSMessage.h",
            "src/chrono_ros/ChROSBridge.h",
            "src/chrono_ros/ChROSHandler.h",
            "src/chrono_ros/ChROSManager.h",
            "src/chrono_ros/ChROSNodeProcess.h",
            "src/chrono_ros/ChApiROS.h",
            "src/chrono_ros/ChROSPublisher.h",
            "src/chrono_ros/ChROSSubscription.h",
            "src/chrono_ros/ChROSQoS.h",
        ],
    },
    "handlers": {
        "sources": [
            "src/chrono_ros/handlers/ChROSClockHandler.cpp",
            "src/chrono_ros/handlers/ChROSBodyHandler.cpp",
            "src/chrono_ros/handlers/ChROSTFHandler.cpp",
            "src/chrono_ros/handlers/robot/ChROSRobotModelHandler.cpp",
        ],
        "headers": [
            "src/chrono_ros/handlers/ChROSClockHandler.h",
            "src/chrono_ros/handlers/ChROSBodyHandler.h",
            "src/chrono_ros/handlers/ChROSTFHandler.h",
            "src/chrono_ros/handlers/robot/ChROSRobotModelHandler.h",
        ],
    },
    "sensor": {
        "sources": [
            "src/chrono_ros/handlers/sensor/ChROSAccelerometerHandler.cpp",
            "src/chrono_ros/handlers/sensor/ChROSGyroscopeHandler.cpp",
            "src/chrono_ros/handlers/sensor/ChROSMagnetometerHandler.cpp",
            "src/chrono_ros/handlers/sensor/ChROSGPSHandler.cpp",
            "src/chrono_ros/handlers/sensor/ChROSIMUHandler.cpp",
        ],
        "headers": [
            "src/chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSGPSHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSIMUHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h",
        ],
    },
    "rendered_sensor": {
        "sources": [
            "src/chrono_ros/handlers/sensor/ChROSCameraHandler.cpp",
            "src/chrono_ros/handlers/sensor/ChROSLidarHandler.cpp",
        ],
        "headers": [
            "src/chrono_ros/handlers/sensor/ChROSCameraHandler.h",
            "src/chrono_ros/handlers/sensor/ChROSLidarHandler.h",
        ],
    },
    "vehicle": {
        "sources": [
            "src/chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.cpp",
        ],
        "headers": [
            "src/chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h",
        ],
    },
    "robot": {
        "sources": [
            "src/chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.cpp",
        ],
        "headers": [
            "src/chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h",
        ],
    },
    "node": {
        "sources": [
            "src/chrono_ros/node/chrono_ros_node.cpp",
            "src/chrono_ros/node/ChROSNodeTypeSupport.cpp",
        ],
        "headers": [
            "src/chrono_ros/node/ChROSNodeTypeSupport.h",
        ],
    },
}
