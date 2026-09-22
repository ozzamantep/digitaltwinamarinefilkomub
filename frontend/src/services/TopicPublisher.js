import { Topic, Service, ServiceRequest } from 'roslib';

class TopicPublisher {
  constructor() {
    this.ros = null;
    this.cmdVelTopic = null;
    this.thrusterCmdTopic = null;
    this.obstacleOrderTopic = null;
    this.gripperCommandTopic = null;
  }

  init(ros) {
    if (!ros) return;
    this.ros = ros;

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

    // 3. Dynamic Obstacle Order & Priority Topic (for Jetson Autonomous Sequencing)
    this.obstacleOrderTopic = new Topic({
      ros,
      name: '/mission_obstacle_order',
      messageType: 'std_msgs/msg/String',
    });

    this.gripperCommandTopic = new Topic({
      ros,
      name: '/gripper/command',
      messageType: 'std_msgs/msg/String',
    });

    console.log('[TopicPublisher] Advertised /cmd_vel, /thruster_commands & /mission_obstacle_order on ROS2');
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

  /**
   * Publish obstacle priority order to Jetson / ROS2 network
   */
  publishObstacleOrder(orderList) {
    if (!this.obstacleOrderTopic || !orderList) return;

    const msg = {
      data: JSON.stringify(orderList),
    };

    this.obstacleOrderTopic.publish(msg);
    console.log('[TopicPublisher] 📡 Published updated obstacle execution order:', orderList);
  }

  publishGripperCommand(command) {
    if (!this.gripperCommandTopic) return;
    this.gripperCommandTopic.publish({ data: command });
  }

  /**
   * Arm/disarm the REAL vehicle via MAVROS (/mavros/cmd/arming).
   */
  armDisarm(value, callback = () => {}) {
    if (!this.ros) return;
    const service = new Service({
      ros: this.ros,
      name: '/mavros/cmd/arming',
      serviceType: 'mavros_msgs/CommandBool',
    });
    service.callService(new ServiceRequest({ value }), callback, (err) => {
      console.error('[TopicPublisher] Arm/disarm service call failed:', err);
    });
  }

  /**
   * Set the REAL vehicle's flight mode via MAVROS (/mavros/set_mode).
   * Only real ArduSub modes (MANUAL, STABILIZE, ALT_HOLD, GUIDED, ...) are valid here -
   * QUALIFIKASI/FINAL are twin-only visualization modes and must never be sent.
   */
  setFlightMode(customMode, callback = () => {}) {
    if (!this.ros) return;
    const service = new Service({
      ros: this.ros,
      name: '/mavros/set_mode',
      serviceType: 'mavros_msgs/SetMode',
    });
    service.callService(new ServiceRequest({ custom_mode: customMode }), callback, (err) => {
      console.error('[TopicPublisher] Set mode service call failed:', err);
    });
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
    if (this.obstacleOrderTopic) {
      this.obstacleOrderTopic.unadvertise();
      this.obstacleOrderTopic = null;
    }
    if (this.gripperCommandTopic) {
      this.gripperCommandTopic.unadvertise();
      this.gripperCommandTopic = null;
    }
  }
}

const topicPublisher = new TopicPublisher();
export default topicPublisher;
