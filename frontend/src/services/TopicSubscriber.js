import { Topic } from 'roslib';
import * as THREE from 'three';
import useVehicleStore from '../store/vehicleStore';
import sysIdEngine from './SystemIdentificationEngine';

class TopicSubscriber {
  constructor() {
    this.subscriptions = [];
    this.rlsActivated = false;
    this._yoloExpiryInterval = null;
    this._yoloDetectionTimestamps = {};
  }

  subscribeAll(ros) {
    this.unsubscribeAll();
    if (!ros) return;

    // ── RESET ALL SENSOR DATA TO 0 WHEN GOING LIVE ────────────────────────
    // Ensures digital twin shows REAL data only — not leftover simulation values.
    // Any sensor that has not yet received a ROS message will display 0.
    const store = useVehicleStore.getState();
    store.updateDepthSensor({ depth: 0, pressure: 0, temperature: 0 });
    store.updateDVL(0);
    store.updateSonarRanges({ front: 0, rear: 0, left: 0, right: 0 });
    store.updateBattery({ level: 0, voltage: 0, current: 0, temperature: 0 });
    store.updateIMU({ roll: 0, pitch: 0, yaw: 0, accelX: 0, accelY: 0, accelZ: 0 });
    store.updateDiagnostics({
      cpuUsage: 0, gpuUsage: 0, memoryUsage: 0,
      diskUsage: 0, rosNodes: 0, topicRate: 0, uptime: 0,
      dvlStatus: 'NO DATA', errors: [],
    });
    console.log('[TopicSubscriber] 🔄 Sensor data reset to 0 — waiting for real Jetson data...');

    // 1. Odometry (Subsea 3D Position & Velocity)
    this.subscribe(ros, '/odom', 'nav_msgs/msg/Odometry', (msg) => {
      const pos = msg.pose.pose.position;
      const ori = msg.pose.pose.orientation;

      // Pool coordinates: X (length), Y (depth in 3D: 2.0 - depth), Z (width)
      const currentDepth = -pos.z > 0 ? -pos.z : 0.8;
      const y3D = 2.0 - currentDepth;

      const surgeVelocity = msg.twist.twist.linear.x;
      const st = useVehicleStore.getState();
      st.setLastOdomTime(Date.now());

      const hasRecentImu = Date.now() - (st.lastImuTime || 0) < 3000;
      st.updatePose({
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
      if (!this.rlsActivated) {
        this.rlsActivated = true;
        sysIdEngine.setRLSEnabled(true);
        console.log('[TopicSubscriber] 🎯 RLS Online Adaptation ACTIVATED');
      }
    });

    // 2. IMU — direct from Pixhawk via MAVROS
    this.subscribe(ros, '/mavros/imu/data', 'sensor_msgs/msg/Imu', (msg) => {
      const q = msg.orientation;
      const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);

      const sinp = 2 * (q.w * q.y - q.z * q.x);
      const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * (180 / Math.PI);

      const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);

      const rollRad = (roll * Math.PI) / 180;
      const pitchRad = (pitch * Math.PI) / 180;
      const yawRad = (yaw * Math.PI) / 180;

      const eulerThree = new THREE.Euler(rollRad, -yawRad, pitchRad, 'YXZ');
      const quat = new THREE.Quaternion().setFromEuler(eulerThree);

      const st = useVehicleStore.getState();
      st.setLastImuTime(Date.now());
      st.updateIMU({
        roll, pitch,
        yaw: (yaw + 360) % 360,
        accelX: msg.linear_acceleration.x,
        accelY: msg.linear_acceleration.y,
        accelZ: msg.linear_acceleration.z,
      });
      st.updateOrientation({ x: quat.x, y: quat.y, z: quat.z, w: quat.w });
      st.setHeadingRad(yawRad);
    });

    // 3. Depth & Pressure Sensor (MS5837)
    this.subscribe(ros, '/depth', 'sensor_msgs/msg/FluidPressure', (msg) => {
      const pressureKPa = msg.fluid_pressure / 1000;
      const depthMeters = (msg.fluid_pressure - 101325) / (997 * 9.81);
      const safeDepth = Math.max(0, depthMeters);
      const st = useVehicleStore.getState();
      st.updateDepthSensor({ depth: safeDepth, pressure: pressureKPa, temperature: 26.5 });
      if (Date.now() - (st.lastOdomTime || 0) > 1000) {
        st.updatePosition({ ...st.position, y: Math.max(0.15, Math.min(1.95, 2.0 - safeDepth)) });
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
    this.subscribe(ros, '/camera/image_raw/compressed', 'sensor_msgs/msg/CompressedImage', (msg) => {
      const imageData = `data:image/jpeg;base64,${msg.data}`;
      useVehicleStore.getState().setCameraFrame(imageData);
    });

    // 7. Live Real-World Thruster Feedback (/thruster_outputs dari manual_bridge.py)
    this.subscribe(ros, '/thruster_outputs', 'std_msgs/msg/Float64MultiArray', (msg) => {
      if (msg.data && msg.data.length >= 6) {
        const thrusterEfforts = msg.data.map((val) => {
          const v = Number(val) || 0;
          return Math.max(-100, Math.min(100, Math.round(v * 100)));
        });
        const thrusterRPMs = thrusterEfforts.map((eff) => Math.round(Math.abs(eff) * 35));
        console.debug('[TopicSubscriber] /thruster_outputs →', thrusterEfforts);
        useVehicleStore.getState().updateThrusters(thrusterEfforts, thrusterRPMs);
      }
    });

    // 7b. Direct MAVROS Physical ESC PWM (/mavros/rc/out)
    this.subscribe(ros, '/mavros/rc/out', 'mavros_msgs/msg/RCOut', (msg) => {
      if (msg.channels && msg.channels.length >= 6) {
        const thrusterEfforts = msg.channels.slice(0, 6).map((pwm) => {
          const val = Number(pwm) || 1500;
          if (val === 0) return 0;
          const effort = (val - 1500) / 400;
          return Math.max(-100, Math.min(100, Math.round(effort * 100)));
        });
        const thrusterRPMs = thrusterEfforts.map((eff) => Math.round(Math.abs(eff) * 35));
        console.debug('[TopicSubscriber] /mavros/rc/out →', thrusterEfforts);
        useVehicleStore.getState().updateThrusters(thrusterEfforts, thrusterRPMs);
      }
    });

    // 8. Dynamic YOLO Obstacle Detection & Real-World Twin Sync
    this.subscribe(ros, '/yolo_target_coord', 'std_msgs/msg/String', (msg) => {
      try {
        const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
        const st = useVehicleStore.getState();

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
          st.setObstaclePos(storeKey, data.x, data.z);
          st.setObstacleDetected(storeKey, true);
          // Record timestamp so expiry interval can clear it after 3s of no detection
          this._yoloDetectionTimestamps[storeKey] = Date.now();
        }
      } catch (e) {
        // Non-JSON string or raw data
      }
    });

    // 8b. YOLO Detection Expiry — mark objects as NOT detected if YOLO stops seeing them.
    // After 3s without a detection message for an object, clear detected: false.
    // This ensures the digital camera clears bounding boxes when real camera loses sight.
    this._yoloExpiryInterval = setInterval(() => {
      const now = Date.now();
      const st = useVehicleStore.getState();
      const obstacles = st.obstacles || {};
      Object.keys(obstacles).forEach((key) => {
        const lastSeen = this._yoloDetectionTimestamps[key] || 0;
        if (obstacles[key]?.detected && now - lastSeen > 3000) {
          st.setObstacleDetected(key, false);
          console.debug(`[TopicSubscriber] YOLO lost sight of ${key}, clearing detection`);
        }
      });
    }, 1000);

    // 9. Real-World Mission State Sync
    this.subscribe(ros, '/mission_state', 'std_msgs/msg/String', (msg) => {
      try {
        const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
        const st = useVehicleStore.getState();

        if (data.event === 'flare_hit' && data.color) {
          st.knockdownFlare(data.color.toLowerCase());
          console.log(`[TopicSubscriber] 🎯 Real Robot hit ${data.color} flare! Digital Twin updated.`);
        }
        if (data.event === 'payload_dropped') {
          st.dropPayload(data.x || 10.5, data.z || 1.5);
          console.log('[TopicSubscriber] 🔴 Real Robot dropped ball payload! Digital Twin updated.');
        }
        if (data.activeTarget) {
          useVehicleStore.setState({ activeTarget: data.activeTarget });
        }
      } catch (e) {
        // Ignore
      }
    });

    // 10. Real Vehicle Arm/Mode Feedback
    this.subscribe(ros, '/mavros/state', 'mavros_msgs/msg/State', (msg) => {
      const st = useVehicleStore.getState();
      st.setArmed(msg.armed);
      if (msg.mode) st.setFlightMode(msg.mode);
    });

    // 11. REALTIME JETSON DIAGNOSTICS — CPU / GPU / RAM / Disk live from Jetson Orin
    // Publishes from jetson_monitor_node running on the Jetson itself via rosbridge.
    // Expected JSON format: { cpu_percent, gpu_percent, ram_percent, disk_percent, ros_nodes, topic_rate_hz, dvl_status }
    this.subscribe(ros, '/jetson/diagnostics', 'std_msgs/msg/String', (msg) => {
      try {
        const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
        useVehicleStore.getState().updateDiagnostics({
          cpuUsage: Number(data.cpu_percent) || 0,
          gpuUsage: Number(data.gpu_percent) || 0,
          memoryUsage: Number(data.ram_percent) || 0,
          diskUsage: Number(data.disk_percent) || 0,
          rosNodes: Number(data.ros_nodes) || 0,
          topicRate: Number(data.topic_rate_hz) || 0,
          dvlStatus: data.dvl_status || 'NO DATA',
        });
      } catch (e) {
        // Ignore malformed messages
      }
    });

    // 11b. Alternative: ROS2 DiagnosticArray format for Jetson stats
    this.subscribe(ros, '/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', (msg) => {
      try {
        const status = msg.status || [];
        const jetsonStatus = status.find((s) => s.name && s.name.toLowerCase().includes('jetson'));
        if (!jetsonStatus) return;
        const vals = {};
        (jetsonStatus.values || []).forEach(({ key, value }) => { vals[key] = value; });
        const updates = {};
        if (vals['cpu_percent'] !== undefined) updates.cpuUsage = parseFloat(vals['cpu_percent']) || 0;
        if (vals['gpu_percent'] !== undefined) updates.gpuUsage = parseFloat(vals['gpu_percent']) || 0;
        if (vals['ram_percent'] !== undefined) updates.memoryUsage = parseFloat(vals['ram_percent']) || 0;
        if (Object.keys(updates).length > 0) useVehicleStore.getState().updateDiagnostics(updates);
      } catch (e) {
        // Ignore
      }
    });

    // 12. Mission Uptime Counter
    this.subscribe(ros, '/mission/uptime', 'std_msgs/msg/Int32', (msg) => {
      useVehicleStore.getState().updateDiagnostics({ uptime: Number(msg.data) || 0 });
    });

    // 13. Hull Leak Detection
    this.subscribe(ros, '/hull_leak', 'std_msgs/msg/Bool', (msg) => {
      useVehicleStore.getState().setLeakDetected(!!msg.data);
    });

    console.log('[TopicSubscriber] ✅ Subscribed to all ROS2 topics: /odom, /mavros/imu, /depth, /dvl, /sonar, /battery_state, /camera, /thruster_outputs, /mavros/rc/out, /yolo_target_coord, /mission_state, /mavros/state, /jetson/diagnostics, /mission/uptime, /hull_leak');
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
    // Clear YOLO detection expiry interval
    if (this._yoloExpiryInterval) {
      clearInterval(this._yoloExpiryInterval);
      this._yoloExpiryInterval = null;
    }
    this._yoloDetectionTimestamps = {};

    this.subscriptions.forEach((topic) => {
      try { topic.unsubscribe(); } catch (e) { /* Ignore */ }
    });
    this.subscriptions = [];
    this.rlsActivated = false;
  }
}

const topicSubscriber = new TopicSubscriber();
export default topicSubscriber;
