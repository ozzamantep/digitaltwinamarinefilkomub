import { useState, useEffect } from 'react';
import useVehicleStore from '../../store/vehicleStore';
import rosConnection from '../../services/RosConnection';
import topicSubscriber from '../../services/TopicSubscriber';
import topicPublisher from '../../services/TopicPublisher';
import mockRos from '../../services/MockRosConnection';

export default function MobileQuickBar() {
  const mode = useVehicleStore((s) => s.mode);
  const jetsonIp = useVehicleStore((s) => s.jetsonIp);
  const armed = useVehicleStore((s) => s.armed);
  const setJetsonIp = useVehicleStore((s) => s.setJetsonIp);
  const setMode = useVehicleStore((s) => s.setMode);
  const setArmed = useVehicleStore((s) => s.setArmed);
  const connectionStatus = useVehicleStore((s) => s.connectionStatus);

  const [ipInput, setIpInput] = useState(jetsonIp || '100.121.102.102');

  useEffect(() => {
    if (jetsonIp) setIpInput(jetsonIp);
  }, [jetsonIp]);

  const handleSwitchMode = (targetMode) => {
    if (targetMode === 'demo') {
      rosConnection.disconnect();
      setMode('demo');
      mockRos.start();
    } else {
      mockRos.stop();
      setMode('live');
      const targetIp = ipInput.trim() || '100.121.102.102';
      setJetsonIp(targetIp);
      const ros = rosConnection.connect(targetIp);
      topicSubscriber.subscribeAll(ros);
      topicPublisher.init(ros);
    }
  };

  const handleConnectToggle = () => {
    if (mode === 'live' && connectionStatus === 'connected') {
      rosConnection.disconnect();
      return;
    }
    mockRos.stop();
    const targetIp = ipInput.trim() || '100.121.102.102';
    setJetsonIp(targetIp);
    setMode('live');
    rosConnection.isManualDisconnect = false;
    const ros = rosConnection.connect(targetIp);
    topicSubscriber.subscribeAll(ros);
    topicPublisher.init(ros);
  };

  const handleToggleArm = () => {
    const nextArmed = !armed;
    setArmed(nextArmed);
    if (mode === 'live') {
      topicPublisher.armDisarm(nextArmed);
    }
  };

  const handleEmergencyStop = () => {
    topicPublisher.emergencyStop();
    setArmed(false);
    if (mode === 'live') {
      topicPublisher.armDisarm(false);
    }
  };

  const isConnected = mode === 'live' && connectionStatus === 'connected';

  return (
    <div className="mobile-only mobile-quick-bar">
      <div className="mobile-quick-row">
        <div className="mode-toggle" style={{ flexShrink: 0 }}>
          <button
            type="button"
            className={`mode-toggle-btn ${mode === 'demo' ? 'active' : ''}`}
            onClick={() => handleSwitchMode('demo')}
          >
            SIM
          </button>
          <button
            type="button"
            className={`mode-toggle-btn ${mode === 'live' ? 'active' : ''}`}
            onClick={() => handleSwitchMode('live')}
          >
            ROS 2
          </button>
        </div>

        <button
          type="button"
          className={`mobile-arm-btn ${armed ? 'armed' : 'disarmed'}`}
          onClick={handleToggleArm}
        >
          {armed ? 'ARMED' : 'DISARMED'}
        </button>

        <button
          type="button"
          className="mobile-estop-btn"
          onClick={handleEmergencyStop}
        >
          E-STOP
        </button>
      </div>

      <div className="mobile-quick-row">
        <input
          type="text"
          className="ip-input mobile-ip-input"
          value={ipInput}
          onChange={(e) => setIpInput(e.target.value)}
          placeholder="100.121.102.102"
          inputMode="decimal"
        />
        <button
          type="button"
          className={`mobile-connect-btn ${isConnected ? 'connected' : ''}`}
          onClick={handleConnectToggle}
        >
          {isConnected ? 'Putus' : 'Connect'}
        </button>
      </div>
    </div>
  );
}
