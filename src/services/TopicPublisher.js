import { Topic } from 'roslib';

class TopicPublisher {
  constructor() {
    this.cmdVelTopic = null;
    this.thrusterCmdTopic = null;
  }

  init(ros) {
    if (!ros) return;

    // 1. Standard ROS2 4-DOF Subsea Velocity Command
    this.cmdVelTopic = new Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
    });

    // 2. Direct 6-Channel Thruster Effort/PWM Command (for Jetson / micro-ROS / PCA9685)
    this.thrusterCmdTopic = new Topic({
      ros,
      name: '/thruster_commands',
      messageType: 'std_msgs/msg/Float64MultiArray',
    });

    console.log('[TopicPublisher] Advertised /cmd_vel & /thruster_commands on ROS2');
  }

  /**
   * Publish subsea velocity command:
   * surge (maju/mundur), sway (geser samping), heave (naik/turun), yaw (belok)
   */
  publishVelocity(surge = 0, sway = 0, heave = 0, yaw = 0) {
    if (!this.cmdVelTopic) return;

    const twist = {
      linear: { x: surge, y: sway, z: heave },
      angular: { x: 0, y: 0, z: yaw },
    };

    this.cmdVelTopic.publish(twist);
  }

  /**
   * Publish direct 6-channel thruster efforts (-100 to +100 or PWM 1100-1900us)
   * [T1_FrontL, T2_FrontR, T3_RearL, T4_RearR, T5_VertFront, T6_VertRear]
   */
  publishThrusters(thrustersArray) {
    if (!this.thrusterCmdTopic || !thrustersArray) return;

    const msg = {
      layout: {
        dim: [{ label: 'thrusters', size: thrustersArray.length, stride: thrustersArray.length }],
        data_offset: 0,
      },
      data: thrustersArray.map((v) => Number((v / 100).toFixed(3))), // Normalized -1.0 to +1.0
    };

    this.thrusterCmdTopic.publish(msg);
  }

  emergencyStop() {
    this.publishVelocity(0, 0, 0, 0);
    this.publishThrusters([0, 0, 0, 0, 0, 0]);
    console.log('[TopicPublisher] 🛑 EMERGENCY STOP SENT: All 6 thrusters halted!');
  }

  cleanup() {
    if (this.cmdVelTopic) {
      this.cmdVelTopic.unadvertise();
      this.cmdVelTopic = null;
    }
    if (this.thrusterCmdTopic) {
      this.thrusterCmdTopic.unadvertise();
      this.thrusterCmdTopic = null;
    }
  }
}

const topicPublisher = new TopicPublisher();
export default topicPublisher;
