#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <Kangaroo.h>

using namespace std;


inline const char* toString(KangarooError err)
{
    switch (err)
    {
        case KANGAROO_NO_ERROR:         return "KANGAROO_NO_ERROR";
        case KANGAROO_NOT_STARTED:      return "KANGAROO_NOT_STARTED";
        case KANGAROO_NOT_HOMED:        return "KANGAROO_NOT_HOMED";
        case KANGAROO_CONTROL_ERROR:    return "KANGAROO_CONTROL_ERROR";
        case KANGAROO_WRONG_MODE:       return "KANGAROO_WRONG_MODE";
        case KANGAROO_SERIAL_TIMEOUT:   return "KANGAROO_SERIAL_TIMEOUT";
        case KANGAROO_TIMED_OUT:        return "KANGAROO_TIMED_OUT";
        default:                        return "Unknown error";
    }
}

class KangarooX2 : public hardware_interface::RobotHW
{
public:
  KangarooX2(string port, unsigned long baud) :
    serial_port(port, baud, serial::Timeout::simpleTimeout(1000)),
    stream(serial_port),
    K(stream),
    K1_fr(K, '1'),
    K2_fl(K, '2'), K3_rr(K, '3', 129), K4_rl(K, '4', 129)
 {

    startChannels();
    //Determined with encoder info instead
         //Ticks to radians conversion: determined by rotating wheel 10 timees,
         //then calling 1,getp over simple serial
    //TODO: implement scaling through KangarooChannel::units call
    //  also implement optional reversal of motors
         //ticksToRadians = 2*M_PI * 10 / 2995; //original implementation

    ticksToRadians = 2*M_PI / 245; //encoder specs for counts
    radiansToTicks = 1/ticksToRadians;

    pos_[0] = 0.0; pos_[1] = 0.0; pos_[2] = 0.0; pos_[3] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0; vel_[2] = 0.0; vel_[3] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0; eff_[2] = 0.0; eff_[3] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0; cmd_[2] = 0.0; cmd_[3] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("front_right_motor_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("front_left_motor_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    hardware_interface::JointStateHandle state_handle_3("rear_right_motor_joint", &pos_[2], &vel_[2], &eff_[2]);
    jnt_state_interface_.registerHandle(state_handle_3);

    hardware_interface::JointStateHandle state_handle_4("rear_left_motor_joint", &pos_[3], &vel_[3], &eff_[3]);
    jnt_state_interface_.registerHandle(state_handle_4);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("front_right_motor_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("front_left_motor_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    hardware_interface::JointHandle vel_handle_3(jnt_state_interface_.getHandle("rear_right_motor_joint"), &cmd_[2]);
    jnt_vel_interface_.registerHandle(vel_handle_3);

    hardware_interface::JointHandle vel_handle_4(jnt_state_interface_.getHandle("rear_left_motor_joint"), &cmd_[3]);
    jnt_vel_interface_.registerHandle(vel_handle_4);

    registerInterface(&jnt_vel_interface_);
  }

  ~KangarooX2(){
    K1_fr.s(0);
    K2_fl.s(0);
    K3_rr.s(0);
    K4_rl.s(0);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1] << ", " << cmd_[2] << ", " << cmd_[3]);
    //send commands to motors
    K1_fr.s((int32_t) cmd_[0] * radiansToTicks);
    K2_fl.s(-(int32_t) cmd_[1] * radiansToTicks);
    K3_rr.s((int32_t) cmd_[2] * radiansToTicks);
    K4_rl.s(-(int32_t) cmd_[3] * radiansToTicks);
  }

  void write(){
    KangarooStatus resultGetK1P = K1_fr.getP();
    KangarooStatus resultGetK2P = K2_fl.getP();
    KangarooStatus resultGetK3P = K3_rr.getP();
    KangarooStatus resultGetK4P = K4_rl.getP();

    bool allOk = resultGetK1P.ok() && resultGetK2P.ok() && resultGetK3P.ok() && resultGetK4P.ok();
    if (!allOk){
        //assume not ok because channels need to be restarted
        //this happens when kangaroo board is reset
        //report error codes, wait a bit, then
        //try restarting channels
        ROS_ERROR_STREAM(
            "Kangaroo status not OK" << endl <<
            "Motor 1 state: " << toString(resultGetK1P.error()) << endl <<
            "Motor 2 state: " << toString(resultGetK2P.error()) << endl<<
            "Motor 3 state: " << toString(resultGetK3P.error()) << endl <<
            "Motor 4 state: " << toString(resultGetK4P.error())
        );

        //warning: before adding the sleep statement below, I burnt out
        //a motor, possbily because of the instant restart on error conditions
        //(
        ros::Duration(10).sleep();
        startChannels();
    }
    else {
        pos_[0] =  resultGetK1P.value() * ticksToRadians;
        pos_[1] = resultGetK2P.value() * ticksToRadians;
        pos_[2] =  resultGetK3P.value() * ticksToRadians;
        pos_[3] = resultGetK4P.value() * ticksToRadians;
        vel_[0] =  K1_fr.getS().value() * ticksToRadians;
        vel_[1] = K2_fl.getS().value() * ticksToRadians;
        vel_[2] =  K3_rr.getS().value() * ticksToRadians;
        vel_[3] = K4_rl.getS().value() * ticksToRadians;
    }
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double ticksToRadians;
  double radiansToTicks;
  double cmd_[4];
  double pos_[4];
  double vel_[4];
  double eff_[4];

  serial::Serial serial_port;
  SerialStream stream;
  KangarooSerial  K;
  KangarooChannel K1_fr;
  KangarooChannel K2_fl;
  KangarooChannel K3_rr;
  KangarooChannel K4_rl;

//set up kangaroo channels
void startChannels() {
  ROS_INFO_STREAM("Starting Kangaroo channels");
  K1_fr.start();
  K2_fl.start();
  K3_rr.start();
  K4_rl.start();
}

};

int main(int argc, char **argv)
{
  double x, y, theta;

  ros::init(argc, argv, "kangaroo_x2");
  ros::NodeHandle nh;

 //TODO: read port and baud as parameters
  //string port = "/dev/ttyACM0";
  //unsigned long baud = 9600;

  string port;
  nh.getParam("/serialPort", port);
  double baud;
  nh.getParam("/baudRate", baud);

  KangarooX2 robot(port, (unsigned long) baud);
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    //TODO: Detect loss of connection to robot and reconnect
    robot.read();
    robot.write();
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
