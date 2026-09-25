import { Topic } from 'roslib';
import * as THREE from 'three';
import useVehicleStore from '../store/vehicleStore';
import sysIdEngine from './SystemIdentificationEngine';

class TopicSubscriber {
  constructor() {
    this.subscriptions = [];
    this.rlsActivated = false; // Track if RLS was enabled for real data
  }

  subscribeAll(ros) {
    this.unsubscribeAll();

    if (!ros) return;

    // 1. Odometry (Subsea 3D Position & Velocity)
    this.subscribe(ros, '/odom', 'nav_msgs/msg/Odometry', (msg) => {
      const pos = msg.pose.pose.position;
      const ori = msg.pose.pose.orientation;

      // Pool coordinates: X (length), Y (depth in 3D: 2.0 - depth), Z (width)
      const currentDepth = -pos.z > 0 ? -pos.z : 0.8;
      const y3D = 2.0 - currentDepth;

      const surgeVelocity = msg.twist.twist.linear.x;
      const store = useVehicleStore.getState();
      store.setLastOdomTime(Date.now());

      const hasRecentImu = Date.now() - (store.lastImuTime || 0) < 3000;
      store.updatePose({
        position: { x: pos.x, y: y3D, z: pos.y },
        ...(hasRecentImu ? {} : { orientation: { x: ori.x, y: ori.z, z: -ori.y, w: ori.w } }),
        depth: currentDepth,
        speed: {
          linear: Math.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2),
          surge: surgeVelocity,
          sway: msg.twist.twist.linear.y,
          heave: msg.twist.twist.linear.z,
          angular: msg.twist.twist.angular.z,
        },
      });

      // Activate RLS online adaptation when first real data arrives from Jetson
      // This makes the digital twin auto-calibrate to match real thruster behavior
      if (!this.rlsActivated) {
        this.rlsActivated = true;
        sysIdEngine.setRLSEnabled(true);
        console.log('[TopicSubscriber] 🎯 RLS Online Adaptation ACTIVATED - Digital twin will auto-calibrate to real thruster data');
      }
    });

    // 2. IMU (Orientation & Inclinometer) - direct from Pixhawk via MAVROS, no standalone IMU sensor
    this.subscribe(ros, '/mavros/imu/data', 'sensor_msgs/msg/Imu', (msg) => {
      const q = msg.orientation;
      // Quaternion to Euler (Roll, Pitch, Yaw)
      const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);

      const sinp = 2 * (q.w * q.y - q.z * q.x);
      const pitch =
        Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * (180 / Math.PI);

      const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);

      const rollRad = (roll * Math.PI) / 180;
      const pitchRad = (pitch * Math.PI) / 180;
      const yawRad = (yaw * Math.PI) / 180;

      // 6-DOF Three.js Quaternion (Y is Up: roll, -yaw around Y, pitch)
      const eulerThree = new THREE.Euler(rollRad, -yawRad, pitchRad, 'YXZ');
      const quat = new THREE.Quaternion().setFromEuler(eulerThree);

      const store = useVehicleStore.getState();
      store.setLastImuTime(Date.now());
      store.updateIMU({
        roll,
        pitch,
        yaw: (yaw + 360) % 360,
        accelX: msg.linear_acceleration.x,
        accelY: msg.linear_acceleration.y,
        accelZ: msg.linear_acceleration.z,
      });

      // Synchronize 3D orientation directly from real Pixhawk IMU
      store.updateOrientation({
        x: quat.x,
        y: quat.y,
        z: quat.z,
        w: quat.w,
      });
      store.setHeadingRad(yawRad);
    });

    // 3. Depth & Pressure Sensor (MS5837 Barometer)
    this.subscribe(ros, '/depth', 'sensor_msgs/msg/FluidPressure', (msg) => {
      const pressureKPa = msg.fluid_pressure / 1000;
      const depthMeters = (msg.fluid_pressure - 101325) / (997 * 9.81);
      const safeDepth = Math.max(0, depthMeters);
      const store = useVehicleStore.getState();
      store.updateDepthSensor({
        depth: safeDepth,
        pressure: pressureKPa,
        temperature: 26.5,
      });
      // Synchronize 3D depth in pool (Y = 2.0 - depth)
      if (Date.now() - (store.lastOdomTime || 0) > 1000) {
        store.updatePosition({
          ...store.position,
          y: Math.max(0.15, Math.min(1.95, 2.0 - safeDepth)),
        });
      }
    });

    // 4. DVL / Sonar Altitude to Bottom
    this.subscribe(ros, '/dvl/range', 'sensor_msgs/msg/Range', (msg) => {
      useVehicleStore.getState().updateDVL(msg.range);
    });

    for (const direction of ['front', 'rear', 'left', 'right']) {
      this.subscribe(ros, `/sonar/${direction}/range`, 'sensor_msgs/msg/Range', (msg) => {
        useVehicleStore.getState().updateSonarRange(direction, msg.range);
      });
    }

    // 5. 4S LiPo Battery State
    this.subscribe(ros, '/battery_state', 'sensor_msgs/msg/BatteryState', (msg) => {
      useVehicleStore.getState().updateBattery({
        level: msg.percentage * 100,
        voltage: msg.voltage,
        current: msg.current,
        temperature: msg.temperature,
      });
    });

    // 6. Subsea Forward Camera (Compressed Stream)
    this.subscribe(
      ros,
      '/camera/image_raw/compressed',
      'sensor_msgs/msg/CompressedImage',
      (msg) => {
        const imageData = `data:image/jpeg;base64,${msg.data}`;
        useVehicleStore.getState().setCameraFrame(imageData);
      }
    );

    // 7. Live Real-World Thruster Feedback (/thruster_outputs dari manual_bridge.py)
    // Format: Float64MultiArray dengan data -1.0..+1.0 per thruster (6 channel)
    this.subscribe(
      ros,
      '/thruster_outputs',
      'std_msgs/msg/Float64MultiArray',
      (msg) => {
        if (msg.data && msg.data.length >= 6) {
          // Convert normalized -1.0..+1.0 → -100..+100 %, PRESERVE SIGN for direction
          const thrusterEfforts = msg.data.map((val) => {
            const v = Number(val) || 0;
            return Math.max(-100, Math.min(100, Math.round(v * 100)));
          });
          // RPM is always positive magnitude; direction carried by effortSign in BlueROV2Model
          const thrusterRPMs = thrusterEfforts.map((eff) => Math.round(Math.abs(eff) * 35));
          console.debug('[TopicSubscriber] /thruster_outputs →', thrusterEfforts);
          useVehicleStore.getState().updateThrusters(thrusterEfforts, thrusterRPMs);
        }
      }
    );

    // 7b. Direct MAVROS Physical ESC PWM (/mavros/rc/out)
    // NOTE: mavros_msgs/msg/RCOut may NOT be registered in rosbridge by default.
    // If this subscription silently fails, /thruster_outputs above (published by
    // manual_bridge.py's rc_out_callback) is the reliable fallback.
    this.subscribe(
      ros,
      '/mavros/rc/out',
      'mavros_msgs/msg/RCOut',
      (msg) => {
        if (msg.channels && msg.channels.length >= 6) {
          // T200 PWM: 1100 = full reverse, 1500 = stop, 1900 = full forward
          const thrusterEfforts = msg.channels.slice(0, 6).map((pwm) => {
            const val = Number(pwm) || 1500;
            // Guard: channels reporting 0 means "not active" — treat as neutral
            if (val === 0) return 0;
            const effort = (val - 1500) / 400; // -1.0 to +1.0
            return Math.max(-100, Math.min(100, Math.round(effort * 100)));
          });
          const thrusterRPMs = thrusterEfforts.map((eff) => Math.round(Math.abs(eff) * 35));
          console.debug('[TopicSubscriber] /mavros/rc/out →', thrusterEfforts);
          useVehicleStore.getState().updateThrusters(thrusterEfforts, thrusterRPMs);
        }
      }
    );

    // 8. Dynamic YOLO Obstacle Detection & Real-World Twin Sync
    this.subscribe(
      ros,
      '/yolo_target_coord',
      'std_msgs/msg/String',
      (msg) => {
        try {
          const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
          const store = useVehicleStore.getState();

          if (data.target && data.x !== undefined && data.z !== undefined) {
            const keyMap = {
              'flare_orange': 'orange_flare',
              'flare_blue': 'blue_flare',
              'flare_red': 'red_flare',
              'flare_yellow': 'yellow_flare',
              'gate': 'gate',
              'drum_red': 'drum_red_tgt',
              'drum_red_1': 'drum_red_tgt',
              'drum_red_2': 'drum_red_2',
              'drum_red_3': 'drum_red_3',
              'red_bucket_2': 'drum_red_2',
              'red_bucket_3': 'drum_red_3',
              'drum_blue': 'drum_blue',
            };
            const storeKey = keyMap[data.target] || data.target;
            store.setObstaclePos(storeKey, data.x, data.z);
            store.setObstacleDetected(storeKey, true);
          }
        } catch (e) {
          // Non-JSON string or raw data
        }
      }
    );

    // 9. Real-World Mission State Sync (Flares Knockdown, Gate Passed, Payload Dropped)
    this.subscribe(
      ros,
      '/mission_state',
      'std_msgs/msg/String',
      (msg) => {
        try {
          const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
          const store = useVehicleStore.getState();

          if (data.event === 'flare_hit' && data.color) {
            store.knockdownFlare(data.color.toLowerCase());
            console.log(`[TopicSubscriber] 🎯 Real Robot hit ${data.color} flare! Digital Twin updated.`);
          }
          if (data.event === 'payload_dropped') {
            store.dropPayload(data.x || 10.5, data.z || 1.5);
            console.log('[TopicSubscriber] 🔴 Real Robot dropped ball payload! Digital Twin updated.');
          }
          if (data.activeTarget) {
            useVehicleStore.setState({ activeTarget: data.activeTarget });
          }
        } catch (e) {
          // Ignore
        }
      }
    );

    // 10. Real Vehicle Arm/Mode Feedback (physical -> digital sync, e.g. safety-switch disarm or QGC mode change)
    this.subscribe(ros, '/mavros/state', 'mavros_msgs/msg/State', (msg) => {
      const store = useVehicleStore.getState();
      store.setArmed(msg.armed);
      if (msg.mode) store.setFlightMode(msg.mode);
    });

    console.log('[TopicSubscriber] Subscribed to BlueROV2 ROS2 subsea topics, /thruster_outputs, /yolo_target_coord & /mission_state');
  }

  subscribe(ros, topicName, messageType, callback) {
    const topic = new Topic({
      ros,
      name: topicName,
      messageType,
      throttle_rate: 80,
    });

    topic.subscribe(callback);
    this.subscriptions.push(topic);
  }

  unsubscribeAll() {
    this.subscriptions.forEach((topic) => {
      try {
        topic.unsubscribe();
      } catch (e) {
        // Ignore
      }
    });
    this.subscriptions = [];
  }
}

const topicSubscriber = new TopicSubscriber();
export default topicSubscriber;
