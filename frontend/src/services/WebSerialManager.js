/**
 * Web Serial Manager & Hardware Bridge for 1x Thruster HIL Testbed
 * 
 * Provides:
 * 1. Direct Web Serial API connection to USB Microcontroller (Arduino/ESP32)
 * 2. Bi-directional data pipeline:
 *    - Digital -> Physical: Sends PWM commands (1300 - 1600 µs, safety-limited)
 *    - Physical -> Digital: Receives live telemetry (RPM, Thrust, Current, Voltage)
 * 3. High-fidelity Virtual Hardware Simulator fallback for offline testing
 */

class WebSerialManager {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.isConnected = false;
    this.baudRate = 115200;
    this.listeners = new Set();

    // Virtual / Hardware Simulator mode
    this.isSimulatingHardware = true;
    this.simulatedPwm = 1500;
    this.simRpm = 0;
    this.simThrust = 0;
    this.simCurrent = 0.25;
    this.simVoltage = 16.0;
    this.simNoiseEnabled = true;
    this.simDefect = 0.0; // 0.0 = perfect, 0.2 = 20% thrust loss (fouled propeller)

    this.simInterval = null;
    this.startHardwareSimulator();
  }

  isSupported() {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  subscribe(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  notify(data) {
    this.listeners.forEach((cb) => {
      try {
        cb(data);
      } catch (err) {
        console.error('Error in WebSerial listener:', err);
      }
    });
  }

  /**
   * Request user to pick a USB COM port via Chrome/Edge Web Serial API
   */
  async connect(baudRate = 115200) {
    if (!this.isSupported()) {
      throw new Error('Web Serial API is not supported in this browser. Please use Chrome, Edge, or Opera.');
    }

    try {
      this.baudRate = baudRate;
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: this.baudRate });
      this.isConnected = true;
      this.isSimulatingHardware = false;

      // Start reader loop
      this.startReading();

      this.notify({ type: 'status', connected: true, port: 'USB Serial (COM)' });
      return true;
    } catch (err) {
      this.isConnected = false;
      console.warn('Web Serial connection cancelled or failed:', err);
      throw err;
    }
  }

  /**
   * Disconnect the serial port
   */
  async disconnect() {
    this.isConnected = false;
    try {
      if (this.reader) {
        await this.reader.cancel();
        this.reader = null;
      }
      if (this.writer) {
        await this.writer.close();
        this.writer = null;
      }
      if (this.port) {
        await this.port.close();
        this.port = null;
      }
    } catch (err) {
      console.warn('Error closing serial port:', err);
    }

    this.notify({ type: 'status', connected: false });
  }

  /**
   * Continuous stream reader loop
   */
  async startReading() {
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = this.port.readable.pipeTo(textDecoder.writable);
    const reader = textDecoder.readable.getReader();
    this.reader = reader;

    let buffer = '';

    try {
      while (this.isConnected) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) {
          buffer += value;
          const lines = buffer.split('\n');
          buffer = lines.pop(); // Retain incomplete line

          for (const line of lines) {
            const clean = line.trim();
            if (clean.length > 0) {
              this.parseIncomingTelemetry(clean);
            }
          }
        }
      }
    } catch (err) {
      console.warn('Serial read error:', err);
    } finally {
      reader.releaseLock();
    }
  }

  /**
   * Parse incoming string from Arduino/microcontroller
   * Supports JSON: {"rpm":2400,"thrust":31.2,"current":4.1,"voltage":16.0}
   * Supports CSV: 2400,31.2,4.1,16.0
   */
  parseIncomingTelemetry(line) {
    try {
      if (line.startsWith('{') && line.endsWith('}')) {
        const data = JSON.parse(line);
        this.notify({
          type: 'telemetry',
          source: 'usb_physical',
          rpmReal: Number(data.rpm || 0),
          thrustReal: Number(data.thrust || 0),
          currentReal: Number(data.current || 0),
          voltageReal: Number(data.voltage || 16.0),
          timestamp: Date.now(),
        });
        return;
      }

      // Try CSV format: RPM, Thrust, Current, Voltage
      const parts = line.split(',').map((p) => parseFloat(p.trim()));
      if (parts.length >= 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
        this.notify({
          type: 'telemetry',
          source: 'usb_physical',
          rpmReal: parts[0] || 0,
          thrustReal: parts[1] || 0,
          currentReal: parts[2] || 0,
          voltageReal: parts[3] || 16.0,
          timestamp: Date.now(),
        });
      }
    } catch (e) {
      console.warn('Failed to parse serial line:', line);
    }
  }

  /**
   * Send PWM command to physical thruster via USB serial
   */
  async sendPwm(pwm) {
    this.simulatedPwm = pwm;

    if (!this.isConnected || !this.port || !this.port.writable) {
      return;
    }

    try {
      const textEncoder = new TextEncoder();
      const writer = this.port.writable.getWriter();
      const message = `PWM:${Math.max(1300, Math.min(1600, Math.round(pwm)))}\n`; // SAFETY: hard clamp 1300-1600 µs
      await writer.write(textEncoder.encode(message));
      writer.releaseLock();
    } catch (err) {
      console.error('Failed to send serial PWM:', err);
    }
  }

  /**
   * High-fidelity Virtual Hardware Simulator
   * Simulates real-world physics with mechanical inertia lag, load cell sensor noise,
   * and optional propeller degradation/cavitation so user can test before plugging in.
   */
  startHardwareSimulator() {
    if (this.simInterval) clearInterval(this.simInterval);

    this.simInterval = setInterval(() => {
      if (this.isConnected && !this.isSimulatingHardware) return;

      const pwm = this.simulatedPwm;
      let targetRpm = 0;

      // Realistic deadband & response (matches bench-calibrated 1492-1508 µs)
      if (pwm > 1508) {
        const delta = (pwm - 1508) * (375 / 392);
        targetRpm = Math.max(150, 10.2 * delta - 0.001 * Math.pow(delta, 2));
      } else if (pwm < 1492) {
        const delta = (1492 - pwm) * (375 / 392);
        targetRpm = -Math.max(150, 9.0 * delta - 0.0009 * Math.pow(delta, 2));
      }

      // Apply mechanical inertia lag (~140ms time constant)
      this.simRpm += 0.28 * (targetRpm - this.simRpm);

      // Hydrodynamic thrust equation with optional simulated blade defect/fouling
      const isForward = this.simRpm >= 0;
      const baseKt = isForward ? 3.48e-6 : 2.80e-6; // Slightly different from DT nominal (realistic physical variation)
      const degradedKt = baseKt * (1.0 - this.simDefect);

      let thrust = degradedKt * this.simRpm * Math.abs(this.simRpm);

      // Add realistic Load Cell sensor noise (~0.25 N Gaussian noise)
      if (this.simNoiseEnabled && Math.abs(this.simRpm) > 50) {
        const noise = (Math.random() - 0.5) * 0.5;
        thrust += noise;
      }

      this.simThrust = thrust;

      // Electrical current draw
      const absRpm = Math.abs(this.simRpm);
      this.simCurrent = 0.28 + Math.pow(absRpm / 1000, 2.5) * 2.4 + (this.simNoiseEnabled ? (Math.random() - 0.5) * 0.1 : 0);

      this.notify({
        type: 'telemetry',
        source: 'hardware_simulator',
        rpmReal: this.simRpm,
        thrustReal: this.simThrust,
        currentReal: Math.max(0, this.simCurrent),
        voltageReal: this.simVoltage,
        timestamp: Date.now(),
      });
    }, 50); // 20 Hz telemetry loop
  }

  setSimulatorDefect(defectFraction) {
    this.simDefect = Math.max(0.0, Math.min(0.5, defectFraction));
  }

  toggleSimulator(active) {
    this.isSimulatingHardware = active;
  }
}

export default new WebSerialManager();
