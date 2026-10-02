"""Explicit retained CMake source groups for the admitted Vehicle profile.

No optional implementation is selected by globbing. Source ownership tests compare
these lists with the original module declarations and the shared SCM/STB owners.
"""

VISUALIZATION_GROUPS = {'vsg': {'sources': ['src/chrono_vehicle/visualization/ChVehicleVisualSystemVSG.cpp',
                     'src/chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.cpp',
                     'src/chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.cpp',
                     'src/chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.cpp',
                     'src/chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.cpp'],
         'headers': ['src/chrono_vehicle/visualization/ChVehicleVisualSystemVSG.h',
                     'src/chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h',
                     'src/chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemVSG.h',
                     'src/chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h',
                     'src/chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemVSG.h']},
 'irrlicht': {'sources': ['src/chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.cpp',
                          'src/chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.cpp',
                          'src/chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.cpp',
                          'src/chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.cpp',
                          'src/chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.cpp'],
              'headers': ['src/chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h',
                          'src/chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h',
                          'src/chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h',
                          'src/chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h',
                          'src/chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.h']}}
