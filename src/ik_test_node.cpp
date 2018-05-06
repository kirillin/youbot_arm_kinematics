#include "kinematics.h"

class IKtest {
 private:
  ros::NodeHandle _nh;
  ros::Publisher pub_js;
  ros::ServiceServer srv_ik;
  Kinematics ks;

 public:
  IKtest() {}
  IKtest(Kinematics _ks): ks(_ks) {
    pub_js = _nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    srv_ik = _nh.advertiseService("ik", &IKtest::handler, this);
  }

  bool handler(youbot_arm_kinematics::IK::Request& req, youbot_arm_kinematics::IK::Response& res) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(req.xyz[0],req.xyz[1],req.xyz[2]));
    tf::Quaternion qt;
    qt.setRPY(req.rpy[0],req.rpy[1],req.rpy[2]);
    transform.setRotation(qt);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ground_link", "tool_link"));

    Vector6d vq; vq << req.xyz[0], req.xyz[1], req.xyz[2], req.rpy[0], req.rpy[1], req.rpy[2];
    ConfigurationsOfManipulator conf_manip = ks.inverse(vq);

    int solve = 0;
    if (conf_manip.solves(solve) == true) {
      cout << std::setprecision(10);
      clog << "qs.row(0): " << conf_manip.qs.row(solve) << endl;

      sensor_msgs::JointState jointState;
      jointState.header.stamp = ros::Time::now();
      jointState.name = {"arm_joint_1", "arm_joint_2", "arm_joint_3",
                         "arm_joint_4", "arm_joint_5",
                         "gripper_finger_joint_l", "gripper_finger_joint_r"};
      for(int i = 0; i < 5; i++) {
        jointState.position.push_back(conf_manip.qs(solve, i));
      }
      jointState.position.push_back(0.015);
      jointState.position.push_back(0.015);
      pub_js.publish(jointState);
      res.achievable = 1;
      return 1;
    }
    res.achievable = 0;
    return 0;
  }

  void spin() {
    ros::Rate R(30);
    while(_nh.ok()) {
      ros::spinOnce();
      R.sleep();
    }
  }

};



int main(int argc, char** argv) {
  ros::init(argc, argv, "ik_test_node");

  VectorNd v1; v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
  VectorNd v2; v2 << M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
  VectorNd v3; v3 << 0.147, 0.0, 0.0, 0.0, 0.218;
  VectorNd v4; v4 << M_PI*169.0/180.0, M_PI*65.0/180.0+M_PI/2, -M_PI*146.0/180.0, M_PI*102.5/180.0+M_PI/2, M_PI*167.5/180.0;
  Kinematics ks(v1, v2, v3, v4);

  IKtest ik(ks);
  ik.spin();


  return 0;
}
