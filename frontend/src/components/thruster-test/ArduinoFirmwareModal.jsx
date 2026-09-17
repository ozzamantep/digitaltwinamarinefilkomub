import { useState } from 'react';

export default function ArduinoFirmwareModal({ isOpen, onClose }) {
  const [copied, setCopied] = useState(false);

  if (!isOpen) return null;

  const arduinoCode = `/**
 * Blue Robotics T200 Single Thruster Digital Twin Firmware
 * Platform: Arduino Uno / Nano / ESP32 / STM32
 * 
 * Hardware Connections:
 * 1. Pin D9 (PWM) -> ESC White Signal Wire
 * 2. GND          -> ESC Black Wire & Battery Ground
 * 3. Pin A5       -> Back-EMF divider dari 1 kabel fasa motor (150k + 10k, TANPA sensor) [RPM_MODE 2]
 *    ATAU Pin D2  -> Hall Effect Sensor A3144 (magnet di hub propeller) [RPM_MODE 1]
 * 4. Pin A0 (DT)  -> HX711 Load Cell Data
 * 5. Pin A1 (SCK) -> HX711 Load Cell Clock
 * 6. Pin A2 (V)   -> Voltage Divider (Optional for Battery V)
 * 7. Pin A3 (I)   -> ACS712 / INA219 (Optional for Current A)
 * 
 * Protocol:
 * - IN (Laptop -> Arduino):  "PWM:1550\\n" (1300 - 1600 us, safety-limited)
 * - OUT (Arduino -> Laptop): {"rpm":2410,"thrust":30.8,"current":3.8,"voltage":16.1}
 */

#include <Servo.h>

Servo esc;
const int ESC_PIN = 9;
const int NEUTRAL_PWM = 1500;

// Variables
int currentPwm = NEUTRAL_PWM;
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // 20 Hz telemetry stream

// ===== Mode Sensor RPM =====
// 0 = DEMO   : RPM disimulasikan dari PWM (tanpa hardware tambahan)
// 1 = HALL   : sensor hall A3144 + magnet di hub propeller (pin D2)
// 2 = BACKEMF: TANPA SENSOR — baca tegangan back-EMF dari 1 kabel fasa motor
//              via pembagi tegangan 150k:10k ke pin A5 (motor BLDC = generator!)
#define RPM_MODE 2

// --- Mode 1: Hall sensor ---
const int HALL_PIN = 2;       // harus pin interrupt (D2 atau D3 di Uno/Nano)
const int PULSES_PER_REV = 1; // jumlah magnet yang ditempel di hub
volatile unsigned long lastPulseUs = 0;
volatile unsigned long pulsePeriodUs = 0;

void onHallPulse() {
  unsigned long nowUs = micros();
  if (lastPulseUs > 0) pulsePeriodUs = nowUs - lastPulseUs;
  lastPulseUs = nowUs;
}

// --- Mode 2: Back-EMF (sensorless) ---
const int BEMF_PIN = A5;
const int MOTOR_POLE_PAIRS = 7; // T200 = motor 14-pole → 7 siklus listrik per 1 putaran propeller
int bemfMid = 0;                // titik tengah sinyal (auto-tracking)
bool bemfWasAbove = false;
unsigned long bemfLastCrossUs = 0;
unsigned long bemfPeriodUs = 0;

void sampleBackEmf() {
  int raw = analogRead(BEMF_PIN);
  bemfMid += (raw - bemfMid) / 64; // pelacak titik tengah lambat
  bool above = raw > bemfMid + 4;  // hysteresis ±4 count anti-noise
  bool below = raw < bemfMid - 4;

  if (above && !bemfWasAbove) {
    // rising crossing = 1 siklus listrik
    unsigned long nowUs = micros();
    if (bemfLastCrossUs > 0) bemfPeriodUs = nowUs - bemfLastCrossUs;
    bemfLastCrossUs = nowUs;
    bemfWasAbove = true;
  } else if (below) {
    bemfWasAbove = false;
  }
}

// Simulated or HX711 load cell readings
float measuredThrust = 0.0; // in Newtons
float measuredRpm = 0.0;
float measuredCurrent = 0.25;
float measuredVoltage = 16.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for serial port to connect

#if RPM_MODE == 1
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHallPulse, FALLING);
#elif RPM_MODE == 2
  #if defined(__AVR__)
  analogReference(INTERNAL); // ref 1.1V: back-EMF putaran jari yang kecil tetap terbaca
  #endif
  pinMode(BEMF_PIN, INPUT);
  bemfMid = analogRead(BEMF_PIN);
#endif

  // Attach ESC
  esc.attach(ESC_PIN, 1100, 1900);
  esc.writeMicroseconds(NEUTRAL_PWM);
  
  // Wait 3 seconds for Blue Robotics Basic ESC arming sequence (beeps)
  delay(3000);
}

void loop() {
  // 1. Read incoming PWM command from Web Serial API
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\\n');
    command.trim();

    if (command.startsWith("PWM:")) {
      int pwmVal = command.substring(4).toInt();
      // SAFETY: hard limit 1300 - 1600 us (full range dapat merusak propeller)
      pwmVal = constrain(pwmVal, 1300, 1600);
      currentPwm = pwmVal;
      esc.writeMicroseconds(currentPwm);
    } else if (command == "STOP") {
      currentPwm = NEUTRAL_PWM;
      esc.writeMicroseconds(NEUTRAL_PWM);
    }
  }

#if RPM_MODE == 2
  sampleBackEmf(); // sampling kontinu tiap iterasi loop (~9 kHz ADC)
#endif

  // 2. Transmit Telemetry back to Digital Twin at 20 Hz
  unsigned long now = millis();
  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = now;

    // TODO: Read your real sensors here
    // Example: measuredThrust = scale.get_units(1) * 9.81; // kg to N
    // Example: measuredCurrent = ina219.getCurrent_mA() / 1000.0;

#if RPM_MODE == 1
    // RPM NYATA dari periode antar-pulsa hall sensor.
    // Ikut terbaca walau propeller diputar manual pakai jari → twin digital ikut berputar.
    noInterrupts();
    unsigned long periodUs = pulsePeriodUs;
    unsigned long lastUs = lastPulseUs;
    interrupts();

    if (periodUs > 0 && (micros() - lastUs) < 1000000UL) {
      measuredRpm = 60000000.0 / ((float)periodUs * PULSES_PER_REV);
    } else {
      measuredRpm = 0; // >1 detik tanpa pulsa = propeller berhenti
    }
    // Satu hall sensor tidak bisa deteksi arah: ambil arah dari perintah PWM
    if (currentPwm < 1476) measuredRpm = -measuredRpm;
    measuredThrust = (measuredRpm >= 0 ? 3.50e-6 : -2.80e-6) * measuredRpm * measuredRpm;
    measuredCurrent = 0.3 + (abs(measuredRpm) / 1000.0) * 1.8;
#elif RPM_MODE == 2
    // RPM NYATA dari back-EMF: motor BLDC yang diputar (oleh ESC ATAU jari)
    // menghasilkan tegangan sinus di kabel fasa → frekuensinya = kecepatan putar.
    if (bemfPeriodUs > 0 && (micros() - bemfLastCrossUs) < 1000000UL) {
      // RPM mekanik = 60e6 / (periode listrik us) / jumlah pole-pair
      measuredRpm = 60000000.0 / ((float)bemfPeriodUs * MOTOR_POLE_PAIRS);
    } else {
      measuredRpm = 0; // >1 detik tanpa siklus = propeller berhenti
    }
    // Back-EMF 1 fasa tidak bisa deteksi arah: ambil arah dari perintah PWM
    if (currentPwm < 1476) measuredRpm = -measuredRpm;
    measuredThrust = (measuredRpm >= 0 ? 3.50e-6 : -2.80e-6) * measuredRpm * measuredRpm;
    measuredCurrent = 0.3 + (abs(measuredRpm) / 1000.0) * 1.8;
#else
    // Mode demo tanpa sensor: perkiraan respons dari PWM
    // (deadband 1476-1524 µs, calibrated: prop starts at 1525 fwd / 1475 rev)
    if (currentPwm > 1524) {
      int delta = currentPwm - 1524;
      measuredRpm = max(150.0, 10.2 * delta * (375.0 / 376.0));
      measuredThrust = (3.50e-6 * measuredRpm * measuredRpm);
      measuredCurrent = 0.3 + (measuredRpm / 1000.0) * 1.8;
    } else if (currentPwm < 1476) {
      int delta = 1476 - currentPwm;
      measuredRpm = -max(150.0, 9.0 * delta * (375.0 / 376.0));
      measuredThrust = -(2.80e-6 * measuredRpm * measuredRpm);
      measuredCurrent = 0.3 + (abs(measuredRpm) / 1000.0) * 1.6;
    } else {
      measuredRpm = 0;
      measuredThrust = 0;
      measuredCurrent = 0.25;
    }
#endif

    // Output JSON string to USB serial
    Serial.print("{\\"rpm\\":");
    Serial.print(measuredRpm, 1);
    Serial.print(",\\"thrust\\":");
    Serial.print(measuredThrust, 2);
    Serial.print(",\\"current\\":");
    Serial.print(measuredCurrent, 2);
    Serial.print(",\\"voltage\\":");
    Serial.print(measuredVoltage, 1);
    Serial.println("}");
  }
}
`;

  const handleCopy = () => {
    navigator.clipboard.writeText(arduinoCode);
    setCopied(true);
    setTimeout(() => setCopied(false), 2500);
  };

  return (
    <div
      style={{
        position: 'fixed',
        inset: 0,
        zIndex: 9999,
        background: 'rgba(0, 0, 0, 0.85)',
        backdropFilter: 'blur(10px)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        padding: '20px',
      }}
      onClick={onClose}
    >
      <div
        style={{
          background: 'var(--bg-secondary, #111827)',
          border: '1px solid rgba(0, 240, 255, 0.3)',
          borderRadius: '16px',
          width: '100%',
          maxWidth: '860px',
          maxHeight: '90vh',
          display: 'flex',
          flexDirection: 'column',
          boxShadow: '0 20px 50px rgba(0, 0, 0, 0.7), 0 0 30px rgba(0, 240, 255, 0.15)',
          overflow: 'hidden',
        }}
        onClick={(e) => e.stopPropagation()}
      >
        {/* Header */}
        <div
          style={{
            padding: '16px 24px',
            borderBottom: '1px solid rgba(148, 163, 184, 0.12)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            background: 'linear-gradient(90deg, rgba(0, 240, 255, 0.08) 0%, transparent 100%)',
          }}
        >
          <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
            <span style={{ fontSize: '1.4rem' }}>⚡</span>
            <div>
              <h3 style={{ margin: 0, color: '#f0f4f8', fontSize: '1.1rem', fontWeight: 700 }}>
                Skema Kabel & Firmware Arduino (Web Serial HIL)
              </h3>
              <p style={{ margin: 0, fontSize: '0.75rem', color: '#94a3b8' }}>
                Jembatan USB antara Laptop / Digital Twin dengan Thruster Fisik Blue Robotics T200
              </p>
            </div>
          </div>
          <button
            onClick={onClose}
            style={{
              background: 'transparent',
              border: 'none',
              color: '#94a3b8',
              fontSize: '1.2rem',
              cursor: 'pointer',
              padding: '4px 8px',
            }}
          >
            ✕
          </button>
        </div>

        {/* Content Body */}
        <div style={{ padding: '20px 24px', overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: '18px' }}>
          {/* Wiring Diagram ASCII / Visual */}
          <div
            style={{
              background: '#0a0e17',
              border: '1px solid rgba(0, 240, 255, 0.2)',
              borderRadius: '10px',
              padding: '14px',
              fontFamily: 'monospace',
              fontSize: '0.78rem',
              color: '#38bdf8',
              lineHeight: 1.5,
              whiteSpace: 'pre',
              overflowX: 'auto',
            }}
          >
{`[LAPTOP]  ══════ USB CABLE ══════► [ARDUINO / ESP32]
                                      │
       ┌──────────────────────────────┼──────────────────────────────┐
       │ Pin D9 (PWM Signal)          │ GND (Common Ground)          │
       ▼                              ▼                              ▼
[ESC White Wire]               [ESC Black Wire]               [Power GND]
       │                              │                              │
       └──────────────┬───────────────┘                              │
                      ▼                                              │
         ┌─────────────────────────┐                                 │
         │ Blue Robotics Basic ESC │◄── Baterai 16V / Power Supply ──┘
         └────────────┬────────────┘
                      ▼ 3-Phase Bullets (A, B, C)
         ┌─────────────────────────┐
         │   Blue Robotics T200    │═════► [Load Cell HX711] ──► Arduino (A0, A1)
         └────────────┬────────────┘
                      │ TANPA SENSOR (RPM_MODE 2): ambil 1 kabel fasa motor
                      │ (bullet A/B/C mana saja) → resistor 150k ─┬─ resistor 10k → GND
                      │                                        └─► Arduino Pin A5
                      │ Motor BLDC = generator: diputar jari pun menghasilkan
                      │ tegangan → RPM nyata terbaca tanpa sensor apa pun!
                      │
                      │ ATAU (RPM_MODE 1): magnet neodymium kecil di hub propeller
                      └─► [Hall Sensor A3144] ──► Arduino Pin D2`}
          </div>

          {/* Quick Setup Instructions */}
          <div
            style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fit, minmax(220px, 1fr))',
              gap: '10px',
              fontSize: '0.75rem',
              color: '#cbd5e1',
            }}
          >
            <div style={{ background: 'rgba(15, 23, 42, 0.6)', padding: '10px', borderRadius: '8px', border: '1px solid rgba(148, 163, 184, 0.1)' }}>
              <strong style={{ color: '#00f0ff' }}>1. Upload Sketch:</strong>
              <p style={{ margin: '4px 0 0 0' }}>Salin kode di bawah ke Arduino IDE, pilih board (Uno/Nano/ESP32), lalu klik Upload.</p>
            </div>
            <div style={{ background: 'rgba(15, 23, 42, 0.6)', padding: '10px', borderRadius: '8px', border: '1px solid rgba(148, 163, 184, 0.1)' }}>
              <strong style={{ color: '#00ff88' }}>2. Hubungkan USB:</strong>
              <p style={{ margin: '4px 0 0 0' }}>Colok kabel USB Arduino ke laptop, lalu klik tombol <strong>"🔌 Connect USB"</strong> di halaman ini.</p>
            </div>
            <div style={{ background: 'rgba(15, 23, 42, 0.6)', padding: '10px', borderRadius: '8px', border: '1px solid rgba(148, 163, 184, 0.1)' }}>
              <strong style={{ color: '#ff8c00' }}>3. Nyalakan Daya:</strong>
              <p style={{ margin: '4px 0 0 0' }}>Nyalakan baterai 16V/PSU ke ESC. ESC akan berbunyi beep konfirmasi arming PWM 1500 µs.</p>
            </div>
          </div>

          {/* Code Section */}
          <div>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '8px' }}>
              <span style={{ fontSize: '0.8rem', fontWeight: 600, color: '#f0f4f8' }}>
                📄 T200_DigitalTwin_Interface.ino (Baud: 115200)
              </span>
              <button
                onClick={handleCopy}
                style={{
                  background: copied ? 'var(--accent-green, #00ff88)' : 'rgba(0, 240, 255, 0.15)',
                  color: copied ? '#000' : 'var(--accent-cyan, #00f0ff)',
                  border: `1px solid ${copied ? '#00ff88' : 'rgba(0, 240, 255, 0.4)'}`,
                  borderRadius: '6px',
                  padding: '5px 12px',
                  fontSize: '0.75rem',
                  fontWeight: 600,
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px',
                  transition: 'all 0.2s ease',
                }}
              >
                {copied ? '✓ Tersalin!' : '📋 Salin Kode Arduino'}
              </button>
            </div>

            <pre
              style={{
                background: '#0a0e17',
                border: '1px solid rgba(148, 163, 184, 0.15)',
                borderRadius: '8px',
                padding: '14px',
                color: '#e2e8f0',
                fontSize: '0.72rem',
                fontFamily: 'monospace',
                maxHeight: '260px',
                overflowY: 'auto',
                margin: 0,
              }}
            >
              {arduinoCode}
            </pre>
          </div>
        </div>

        {/* Footer */}
        <div
          style={{
            padding: '12px 24px',
            borderTop: '1px solid rgba(148, 163, 184, 0.12)',
            display: 'flex',
            justifyContent: 'flex-end',
            background: 'rgba(11, 15, 25, 0.95)',
          }}
        >
          <button
            onClick={onClose}
            style={{
              background: 'rgba(148, 163, 184, 0.15)',
              border: '1px solid rgba(148, 163, 184, 0.2)',
              color: '#f0f4f8',
              borderRadius: '6px',
              padding: '6px 18px',
              fontSize: '0.8rem',
              cursor: 'pointer',
              fontWeight: 600,
            }}
          >
            Tutup
          </button>
        </div>
      </div>
    </div>
  );
}
