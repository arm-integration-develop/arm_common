//
// Created by lsy on 23-9-11.
//

#include "arm_common/tools/ros_param.h"
#include "arm_common/tools//can_motor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_can");
  ros::NodeHandle nh("test_can");

  ros::NodeHandle nh_test_can(nh, "test_can");
  ros::Rate loop_rate(100);
  can_interface::CanMotor can_motor;
  XmlRpc::XmlRpcValue xml_rpc_value;

  if (nh.hasParam("actuator_coefficient"))
  {
    ROS_INFO_STREAM("actuator_coefficient");
    nh.getParam("actuator_coefficient", xml_rpc_value);
    can_motor.parseActCoeffs(xml_rpc_value);
  }
  else
  {
    ROS_INFO_STREAM("NO such actuator_coefficient");
  }

  if (nh_test_can.hasParam("actuators"))
  {
    ROS_INFO_STREAM("actuators");
    nh_test_can.getParam("actuators", xml_rpc_value);
    can_motor.parseActData(xml_rpc_value, nh_test_can);
  }
  else
  {
    ROS_INFO_STREAM("NO such actuators");
  }
  can_motor.initCanBus(nh_test_can);
  ROS_INFO_STREAM("Start Init");
  can_motor.can_buses_[0]->start();
  ROS_INFO_STREAM("Init OK");

  while (ros::ok())
  {
    ros::spinOnce();
    for (auto bus : can_motor.can_buses_)
    {
      bus->test();
      bus->read(ros::Time::now());
    }
    loop_rate.sleep();
  }
  for (auto bus : can_motor.can_buses_)
  {
    bus->close();
    ROS_INFO_STREAM("CLOSE");
  }
  return 0;
}