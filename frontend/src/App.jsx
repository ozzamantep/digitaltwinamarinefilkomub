import Header from './components/dashboard/Header';
import SensorPanel from './components/dashboard/SensorPanel';
import JetsonTerminalBox from './components/dashboard/JetsonTerminalBox';
import VehicleScene from './components/3d/VehicleScene';
import ChartPanel from './components/dashboard/ChartPanel';
import CameraFeed from './components/dashboard/CameraFeed';
import ControlPanel from './components/dashboard/ControlPanel';
import DiagnosticsPanel from './components/dashboard/DiagnosticsPanel';
import ThrusterTestbed from './components/thruster-test/ThrusterTestbed';
import useVehicleStore from './store/vehicleStore';

export default function App() {
  const activePage = useVehicleStore((s) => s.activePage);

  return (
    <div className="app-layout">
      <Header />

      {activePage === 'thruster_test' ? (
        <ThrusterTestbed />
      ) : (
        <div className="app-body">
          {/* Left Sidebar - Sensor Data & Terminal Command Box */}
          <div className="left-sidebar">
            <SensorPanel />
            <JetsonTerminalBox />
          </div>

          {/* Center - 3D Viewport + Charts */}
          <div className="center-area">
            <VehicleScene />
            <ChartPanel />
          </div>

          {/* Right Sidebar - Camera, Controls, Diagnostics */}
          <div className="right-sidebar">
            <CameraFeed />
            <ControlPanel />
            <DiagnosticsPanel />
          </div>
        </div>
      )}
    </div>
  );
}
