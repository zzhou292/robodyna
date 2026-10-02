#pragma SWIG nowarn=401
#pragma SWIG nowarn=503

%module(directors="1") chrono_robot
// RoboSimian shapes are distinct native types from the Core geometry shapes.
%rename(RoboSimianBoxShape) chrono::robosimian::BoxShape;
%rename(RoboSimianCylinderShape) chrono::robosimian::CylinderShape;
%rename(RoboSimianSphereShape) chrono::robosimian::SphereShape;
%include "chrono_swig/interface/robot/ChModuleRobot.i"
