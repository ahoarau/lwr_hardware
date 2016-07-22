#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

// Start of user code includes
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <kuka_lwr_fri/friComm.h>

#include <lwr_fri/CartesianImpedance.h>
#include <lwr_fri/FriJointImpedance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>

#ifndef HAVE_RTNET

#define rt_dev_socket socket
#define rt_dev_setsockopt setsockopt
#define rt_dev_bind bind
#define rt_dev_recvfrom recvfrom
#define rt_dev_sendto sendto
#define rt_dev_close close

#else
#include <rtdm/rtdm.h>
#endif

// End of user code

class FRIComponent : public RTT::TaskContext {
public:
  FRIComponent(const std::string & name) : TaskContext(name),robot_name(name),n_joints(LBR_MNJ), prop_joint_offset(LBR_MNJ, 0.0) {

    prop_fri_port = 49938;

    this->addProperty("fri_port", prop_fri_port);
    this->addProperty("joint_offset", prop_joint_offset);
    this->addProperty("robot_name",robot_name);
    this->addProperty("n_joints",n_joints);

    this->ports()->addPort("CartesianImpedanceCommand", port_CartesianImpedanceCommand).doc("");
    this->ports()->addPort("CartesianWrenchCommand", port_CartesianWrenchCommand).doc("");
    this->ports()->addPort("CartesianPositionCommand", port_CartesianPositionCommand).doc("");
    this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc("");
    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");
    this->ports()->addPort("toKRL",port_ToKRL).doc("");
    //this->ports()->addPort("KRL_CMD", port_KRL_CMD).doc("");
    this->ports()->addPort("fromKRL",port_FromKRL).doc("");
    this->ports()->addPort("CartesianWrench", port_CartesianWrench).doc("");
    this->ports()->addPort("RobotState", port_RobotState).doc("");
    this->ports()->addPort("FRIState", port_FRIState).doc("");
    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
    this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc("");
    this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
    this->ports()->addPort("Jacobian", port_Jacobian).doc("");
    this->ports()->addPort("JointTorque", port_JointTorque).doc("");
    this->ports()->addPort("JointTorqueAct", port_JointTorqueAct).doc("");
    this->ports()->addPort("GravityTorque", port_GravityTorque);
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
    this->ports()->addPort("JointPositionFRIOffset", port_JointPositionFRIOffset).doc("");
  }

  ~FRIComponent(){
  }

  bool configureHook() {
    // Start of user code configureHook
    jnt_pos.setZero(LBR_MNJ);
    jnt_pos_fri_off.setZero(LBR_MNJ);
    jnt_pos_old.setZero(LBR_MNJ);
    jnt_vel.setZero(LBR_MNJ);
    jnt_trq.setZero(LBR_MNJ);
    jnt_trq_act.setZero(LBR_MNJ);
    grav_trq.setZero(LBR_MNJ);
    jnt_pos_cmd.setZero(LBR_MNJ);
    jnt_trq_cmd.setZero(LBR_MNJ);
    jac.resize(LBR_MNJ);
    SetToZero(jac);
    mass.setZero(LBR_MNJ,LBR_MNJ);

    port_JointPosition.setDataSample(jnt_pos);
    port_JointPositionFRIOffset.setDataSample(jnt_pos_fri_off);
    port_JointVelocity.setDataSample(jnt_vel);
    port_JointTorque.setDataSample(jnt_trq);
    port_JointTorqueAct.setDataSample(jnt_trq_act);
    port_GravityTorque.setDataSample(grav_trq);
    port_Jacobian.setDataSample(jac);
    port_MassMatrix.setDataSample(mass);

    if (fri_create_socket() != 0)
      return false;
    // End of user code
    return true;
  }

  bool startHook() {
    // Start of user code startHook
    // End of user code
    return true;
  }

  void stopHook() {
    // Start of user code stopHook
    // End of user code
  }

  void updateHook() {
    if( true) {
      doComm();
    }
  }

private:

  void doComm() {
    // Start of user code Comm

    //Read:
    if (fri_recv() == 0) {

      baseFrame.M = KDL::Rotation::RPY(m_msr_data.krl.realData[3] * M_PI / 180.0,
              m_msr_data.krl.realData[4] * M_PI / 180.0,
              m_msr_data.krl.realData[5] * M_PI / 180.0);

      for(unsigned int i=0; i<3; ++i)
        baseframe_vector[i] = m_msr_data.krl.realData[i] / 1000.0;

      baseFrame.p = baseframe_vector;

      // Fill in fri_joint_state and joint_state
      for (unsigned int i = 0; i < LBR_MNJ; i++) {
        grav_trq[i] = m_msr_data.data.gravity[i];
        jnt_trq[i] = m_msr_data.data.estExtJntTrq[i];
        jnt_trq_act[i] = m_msr_data.data.msrJntTrq[i];
        jnt_pos_fri_off[i] = m_msr_data.data.cmdJntPosFriOffset[i];
        jnt_pos[i] = m_msr_data.data.msrJntPos[i] + prop_joint_offset[i];
        jnt_vel[i] = (jnt_pos[i] - jnt_pos_old[i]) / m_msr_data.intf.desiredMsrSampleTime;
        jnt_pos_old[i] = jnt_pos[i];
      }

      cartPos.M = KDL::Rotation(m_msr_data.data.msrCartPos[0],
          m_msr_data.data.msrCartPos[1], m_msr_data.data.msrCartPos[2],
          m_msr_data.data.msrCartPos[4], m_msr_data.data.msrCartPos[5],
          m_msr_data.data.msrCartPos[6], m_msr_data.data.msrCartPos[8],
          m_msr_data.data.msrCartPos[9], m_msr_data.data.msrCartPos[10]);

      cartPos.p.x(m_msr_data.data.msrCartPos[3]);
      cartPos.p.y(m_msr_data.data.msrCartPos[7]);
      cartPos.p.z(m_msr_data.data.msrCartPos[11]);
      cartPos = baseFrame * cartPos;
      tf::poseKDLToMsg(cartPos, cart_pos);

      v = KDL::diff(T_old, cartPos, m_msr_data.intf.desiredMsrSampleTime);
      v = cartPos.M.Inverse() * v;
      T_old = cartPos;
      tf::twistKDLToMsg(v, cart_twist);

      cart_wrench.force.x = m_msr_data.data.estExtTcpFT[0];
      cart_wrench.force.y = m_msr_data.data.estExtTcpFT[1];
      cart_wrench.force.z = m_msr_data.data.estExtTcpFT[2];
      cart_wrench.torque.x = m_msr_data.data.estExtTcpFT[5];
      cart_wrench.torque.y = m_msr_data.data.estExtTcpFT[4];
      cart_wrench.torque.z = m_msr_data.data.estExtTcpFT[3];

      for (int i = 0; i < FRI_CART_VEC; i++)
        for (int j = 0; j < LBR_MNJ; j++)
          jac(i, j) = m_msr_data.data.jacobian[i * LBR_MNJ + j];
      //Kuka uses Tx, Ty, Tz, Rz, Ry, Rx convention, so we need to swap Rz and Rx
      jac.data.row(3).swap(jac.data.row(5));

      for (unsigned int i = 0; i < LBR_MNJ; i++) {
        for (unsigned int j = 0; j < LBR_MNJ; j++) {
          mass(i, j) = m_msr_data.data.massMatrix[LBR_MNJ * i + j];
        }
      }

      //Fill in datagram to send:
      m_cmd_data.head.datagramId = FRI_DATAGRAM_ID_CMD;
      m_cmd_data.head.packetSize = sizeof(tFriCmdData);
      m_cmd_data.head.sendSeqCount = ++counter;
      m_cmd_data.head.reflSeqCount = m_msr_data.head.sendSeqCount;

      //Process KRL CMD
      if (port_ToKRL.read(m_toKRL) != RTT::NoData) {         
        for(int i=0 ; i < FRI_USER_SIZE ; ++i)
        {
            m_cmd_data.krl.intData[i] = m_toKRL.intData[i];
            m_cmd_data.krl.realData[i] = m_toKRL.realData[i];
        }

        m_cmd_data.krl.boolData = m_toKRL.boolData;
      }

      if (!isPowerOn()) {
        // necessary to write cmd if not powered on. See kuka FRI user manual p6 and friremote.cpp:
        for (int i = 0; i < LBR_MNJ; i++) {
          m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i]
              + m_msr_data.data.cmdJntPosFriOffset[i];
        }
      }
      if (m_msr_data.intf.state == FRI_STATE_MON || !isPowerOn()) {
        // joint position control capable modes:
        if (m_msr_data.robot.control == FRI_CTRL_POSITION
            || m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS;
          for (unsigned int i = 0; i < LBR_MNJ; i++) {
            // see note above with !isPowerOn()
            // the user manual speaks of 'mimic msr.data.msrCmdJntPos' which is ambiguous.
            // on the other hand, the friremote.cpp will send this whenever (!isPowerOn() || state != FRI_STATE_CMD)
            // so we mimic the kuka reference code here...
            m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i]
                + m_msr_data.data.cmdJntPosFriOffset[i];
          }
        }
        // Additional flags are set in joint impedance mode:
        if (m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_JNTTRQ;
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_JNTSTIFF | FRI_CMD_JNTDAMP;
          for (unsigned int i = 0; i < LBR_MNJ; i++) {
            m_cmd_data.cmd.addJntTrq[i] = 0.0;
            m_cmd_data.cmd.jntStiffness[i] = 1000;
            m_cmd_data.cmd.jntDamping[i] = 0.7;
          }
        }
        if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
          m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS | FRI_CMD_TCPFT;
          m_cmd_data.cmd.cmdFlags |= FRI_CMD_CARTSTIFF | FRI_CMD_CARTDAMP;
          for (unsigned int i = 0; i < FRI_CART_FRM_DIM; i++)
            m_cmd_data.cmd.cartPos[i] = m_msr_data.data.msrCartPos[i];
          for (unsigned int i = 0; i < FRI_CART_VEC; i++)
            m_cmd_data.cmd.addTcpFT[i] = 0.0;
          for (unsigned int i = 0; i < FRI_CART_VEC / 2; i++) {
            //Linear part;
            m_cmd_data.cmd.cartStiffness[i] = 2000;
            m_cmd_data.cmd.cartDamping[i] = 0.7;
            //rotational part;
            m_cmd_data.cmd.cartStiffness[i + FRI_CART_VEC / 2] = 200;
            m_cmd_data.cmd.cartDamping[i + FRI_CART_VEC / 2] = 0.7;
          }
        }
      }
      //Only send if state is in FRI_STATE_CMD and drives are powerd
      if ((m_msr_data.intf.state == FRI_STATE_CMD) && isPowerOn()) {
        //Valid ports in joint position and joint impedance mode
        if (m_msr_data.robot.control == FRI_CTRL_POSITION
            || m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          //Read desired positions
          if (port_JointPositionCommand.read(jnt_pos_cmd) == RTT::NewData) {
            if (jnt_pos_cmd.size() == LBR_MNJ) {
              for (unsigned int i = 0; i < LBR_MNJ; i++)
                m_cmd_data.cmd.jntPos[i] = jnt_pos_cmd[i];
            } else
              RTT::log(RTT::Warning) << "Size of " << port_JointPositionCommand.getName()
                  << " not equal to " << LBR_MNJ << RTT::endlog();

          }
        }
        //Valid ports only in joint impedance mode
        if (m_msr_data.robot.control == FRI_CTRL_JNT_IMP) {
          //Read desired additional joint torques
          if (port_JointTorqueCommand.read(jnt_trq_cmd)
              == RTT::NewData) {
            //Check size
            if (jnt_trq_cmd.size() == LBR_MNJ) {
              for (unsigned int i = 0; i < LBR_MNJ; i++)
                m_cmd_data.cmd.addJntTrq[i] = jnt_trq_cmd[i];
            } else
              RTT::log(RTT::Warning) << "Size of " << port_JointTorqueCommand.getName()
                  << " not equal to " << LBR_MNJ << RTT::endlog();

          }
          //Read desired joint impedance
          if (port_JointImpedanceCommand.read(jnt_imp_cmd) == RTT::NewData) {
            for (unsigned int i = 0; i < LBR_MNJ; i++) {
              m_cmd_data.cmd.jntStiffness[i] =
                  jnt_imp_cmd.stiffness[i];
              m_cmd_data.cmd.jntDamping[i] = jnt_imp_cmd.damping[i];
            }
          }
        } else if (m_msr_data.robot.control == FRI_CTRL_CART_IMP) {
          if (port_CartesianPositionCommand.read(cart_pos_cmd) == RTT::NewData) {
            rot = KDL::Rotation::Quaternion(
                cart_pos_cmd.orientation.x, cart_pos_cmd.orientation.y,
                cart_pos_cmd.orientation.z, cart_pos_cmd.orientation.w);
            m_cmd_data.cmd.cartPos[0] = rot.data[0];
            m_cmd_data.cmd.cartPos[1] = rot.data[1];
            m_cmd_data.cmd.cartPos[2] = rot.data[2];
            m_cmd_data.cmd.cartPos[4] = rot.data[3];
            m_cmd_data.cmd.cartPos[5] = rot.data[4];
            m_cmd_data.cmd.cartPos[6] = rot.data[5];
            m_cmd_data.cmd.cartPos[8] = rot.data[6];
            m_cmd_data.cmd.cartPos[9] = rot.data[7];
            m_cmd_data.cmd.cartPos[10] = rot.data[8];

            m_cmd_data.cmd.cartPos[3] = cart_pos_cmd.position.x;
            m_cmd_data.cmd.cartPos[7] = cart_pos_cmd.position.y;
            m_cmd_data.cmd.cartPos[11] = cart_pos_cmd.position.z;
          }

          if (port_CartesianWrenchCommand.read(cart_wrench_cmd) == RTT::NewData) {
            m_cmd_data.cmd.addTcpFT[0] = cart_wrench_cmd.force.x;
            m_cmd_data.cmd.addTcpFT[1] = cart_wrench_cmd.force.y;
            m_cmd_data.cmd.addTcpFT[2] = cart_wrench_cmd.force.z;
            m_cmd_data.cmd.addTcpFT[3] = cart_wrench_cmd.torque.z;
            m_cmd_data.cmd.addTcpFT[4] = cart_wrench_cmd.torque.y;
            m_cmd_data.cmd.addTcpFT[5] = cart_wrench_cmd.torque.x;
          }

          if (port_CartesianImpedanceCommand.read(cart_imp_cmd) == RTT::NewData) {
            m_cmd_data.cmd.cartStiffness[0] = cart_imp_cmd.stiffness.linear.x;
            m_cmd_data.cmd.cartStiffness[1] = cart_imp_cmd.stiffness.linear.y;
            m_cmd_data.cmd.cartStiffness[2] = cart_imp_cmd.stiffness.linear.z;
            m_cmd_data.cmd.cartStiffness[5] = cart_imp_cmd.stiffness.angular.x;
            m_cmd_data.cmd.cartStiffness[4] = cart_imp_cmd.stiffness.angular.y;
            m_cmd_data.cmd.cartStiffness[3] = cart_imp_cmd.stiffness.angular.z;
            m_cmd_data.cmd.cartDamping[0] = cart_imp_cmd.damping.linear.x;
            m_cmd_data.cmd.cartDamping[1] = cart_imp_cmd.damping.linear.y;
            m_cmd_data.cmd.cartDamping[2] = cart_imp_cmd.damping.linear.z;
            m_cmd_data.cmd.cartDamping[5] = cart_imp_cmd.damping.angular.x;
            m_cmd_data.cmd.cartDamping[4] = cart_imp_cmd.damping.angular.y;
            m_cmd_data.cmd.cartDamping[3] = cart_imp_cmd.damping.angular.z;
          }
        } else if (m_msr_data.robot.control == FRI_CTRL_OTHER) {
          this->error();
        }
      }						//End command mode

      //Put robot and fri state on the ports(no parsing)
      port_RobotState.write(m_msr_data.robot);
      port_FRIState.write(m_msr_data.intf);

      port_JointPosition.write(jnt_pos);
      port_JointPositionFRIOffset.write(jnt_pos_fri_off);
      port_JointVelocity.write(jnt_vel);
      port_JointTorque.write(jnt_trq);
      port_JointTorqueAct.write(jnt_trq_act);
      port_GravityTorque.write(grav_trq);

      port_CartesianPosition.write(cart_pos);
      port_CartesianVelocity.write(cart_twist);
      port_CartesianWrench.write(cart_wrench);

      port_Jacobian.write(jac);
      port_MassMatrix.write(mass);

      port_FromKRL.write(m_msr_data.krl);

      if(fri_send() != 0)
        this->error();
    }

    this->trigger();
    // End of user code
  }


  RTT::InputPort<lwr_fri::CartesianImpedance > port_CartesianImpedanceCommand;
  RTT::InputPort<geometry_msgs::Wrench > port_CartesianWrenchCommand;
  RTT::InputPort<geometry_msgs::Pose > port_CartesianPositionCommand;
  RTT::InputPort<lwr_fri::FriJointImpedance > port_JointImpedanceCommand;
  RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
  RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;
  RTT::InputPort<tFriKrlData > port_ToKRL;
  //RTT::InputPort<std_msgs::Int32 > port_KRL_CMD;

  RTT::OutputPort<tFriKrlData > port_FromKRL;
  RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
  RTT::OutputPort<tFriRobotState > port_RobotState;
  RTT::OutputPort<tFriIntfState > port_FRIState;
  RTT::OutputPort<Eigen::VectorXd > port_JointVelocity;
  RTT::OutputPort<geometry_msgs::Twist > port_CartesianVelocity;
  RTT::OutputPort<geometry_msgs::Pose > port_CartesianPosition;
  RTT::OutputPort<Eigen::MatrixXd > port_MassMatrix;
  RTT::OutputPort<KDL::Jacobian > port_Jacobian;
  RTT::OutputPort<Eigen::VectorXd > port_JointTorque;
  RTT::OutputPort<Eigen::VectorXd > port_JointTorqueAct;
  RTT::OutputPort<Eigen::VectorXd > port_GravityTorque;
  RTT::OutputPort<Eigen::VectorXd > port_JointPosition;
  RTT::OutputPort<Eigen::VectorXd > port_JointPositionFRIOffset;

  int prop_fri_port;
  std::string robot_name;
  unsigned int n_joints;
  std::vector<double> prop_joint_offset;

  // Start of user code userData
  Eigen::VectorXd jnt_pos;
  Eigen::VectorXd jnt_pos_fri_off;
  Eigen::VectorXd jnt_pos_old;
  Eigen::VectorXd jnt_trq;
  Eigen::VectorXd jnt_trq_act;
  Eigen::VectorXd grav_trq;
  Eigen::VectorXd jnt_vel;

  Eigen::VectorXd jnt_pos_cmd;
  Eigen::VectorXd jnt_trq_cmd;

  geometry_msgs::Pose cart_pos, cart_pos_cmd;
  geometry_msgs::Wrench cart_wrench, cart_wrench_cmd;
  geometry_msgs::Twist cart_twist;
  Eigen::MatrixXd mass;
  lwr_fri::FriJointImpedance jnt_imp_cmd;
  lwr_fri::CartesianImpedance cart_imp_cmd;
  KDL::Frame cartPos;
  KDL::Rotation rot;
  KDL::Frame baseFrame;
  KDL::Vector baseframe_vector;
  KDL::Rotation baseframe_rotation;
  std_msgs::Int32 x;
  KDL::Jacobian jac;
  KDL::Twist v;
  KDL::Frame T_old;

  int m_socket, m_remote_port, m_control_mode;
  std::string joint_names_prefix;
  uint16_t counter, fri_state_last;
  struct sockaddr_in m_remote_addr;
  socklen_t m_sock_addr_len;

  tFriMsrData m_msr_data;
  tFriCmdData m_cmd_data;
  tFriKrlData m_fromKRL;
  tFriKrlData m_toKRL;

  int fri_recv() {
    int n = rt_dev_recvfrom(m_socket, (void*) &m_msr_data, sizeof(m_msr_data),
        0, (sockaddr*) &m_remote_addr, &m_sock_addr_len);
    if (sizeof(tFriMsrData) != n) {
      RTT::log(RTT::Error) << "bad packet length: " << n << ", expected: "
          << sizeof(tFriMsrData) << RTT::endlog();
      return -1;
    }
    return 0;
  }

  int fri_send() {
    int ret = rt_dev_sendto(m_socket, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
            (sockaddr*) &m_remote_addr, m_sock_addr_len);
    if(ret <=0)
    {
      RTT::log(RTT::Error) << "Sending datagram failed. (ret="<<ret<<")"
          << ntohs(m_remote_addr.sin_port) << RTT::endlog();
      return -1;
    }
    return 0;
  }

  bool isPowerOn() { return m_msr_data.robot.power!=0; }

  int fri_create_socket() {

    if (m_socket != 0)
      rt_dev_close(m_socket);
    m_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    rt_dev_setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

    struct sockaddr_in local_addr;
    bzero((char *) &local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(prop_fri_port);

    if (rt_dev_bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
      RTT::log(RTT::Error) << "Binding of port failed with errno " << errno << RTT::endlog();
      return -1;
    }

    return 0;
  }
  // End of user code

};

ORO_CREATE_COMPONENT(FRIComponent)
