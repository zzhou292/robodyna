"""All robot demo mains, CMake declarations and retained local helper inputs."""

ROBOT_DEMO_SOURCES = ['src/demos/robot/curiosity/demo_ROBOT_Curiosity_Rigid.cpp',
 'src/demos/robot/curiosity/demo_ROBOT_Curiosity_SCM.cpp',
 'src/demos/robot/curiosity/demo_ROBOT_Curiosity_SCM_Sensor.cpp',
 'src/demos/robot/hexy/demo_ROBOT_LittleHexy.cpp',
 'src/demos/robot/industrial/demo_ROBOT_Industrial.cpp',
 'src/demos/robot/lander/demo_ROBOT_Lander.cpp',
 'src/demos/robot/lander/demo_ROBOT_Lander_CRM.cpp',
 'src/demos/robot/robosimian/multicore/demo_ROBOT_RoboSimian_Granular.cpp',
 'src/demos/robot/robosimian/sequential/demo_ROBOT_RoboSimian_Rigid.cpp',
 'src/demos/robot/robosimian/sequential/demo_ROBOT_RoboSimian_SCM.cpp',
 'src/demos/robot/turtlebot/demo_ROBOT_Turtlebot_Rigid.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_CRM.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_Rigid.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_SCM.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_SCM_Sensor.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_WheelSlope_CRM.cpp',
 'src/demos/robot/viper/demo_ROBOT_Viper_WheelTestRig_CRM.cpp']

ROBOT_DEMO_HEADERS = ['src/demos/SetChronoSolver.h',
 'src/demos/robot/lander/model/Lander.h',
 'src/demos/robot/robosimian/multicore/granular.h',
 'src/demos/robot/viper/viper_wheel.h']

ROBOT_DEMO_CMAKE_FILES = ['src/demos/robot/CMakeLists.txt',
 'src/demos/robot/curiosity/CMakeLists.txt',
 'src/demos/robot/hexy/CMakeLists.txt',
 'src/demos/robot/industrial/CMakeLists.txt',
 'src/demos/robot/lander/CMakeLists.txt',
 'src/demos/robot/robosimian/CMakeLists.txt',
 'src/demos/robot/robosimian/multicore/CMakeLists.txt',
 'src/demos/robot/robosimian/sequential/CMakeLists.txt',
 'src/demos/robot/turtlebot/CMakeLists.txt',
 'src/demos/robot/viper/CMakeLists.txt']

ROBOT_DEMO_HELPERS = ['src/demos/robot/lander/model/Lander.cpp',
 'src/demos/robot/lander/model/Lander.h',
 'src/demos/robot/robosimian/multicore/granular.cpp',
 'src/demos/robot/robosimian/multicore/granular.h']
