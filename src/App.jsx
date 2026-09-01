import Header from './components/dashboard/Header';
import SensorPanel from './components/dashboard/SensorPanel';
import VehicleScene from './components/3d/VehicleScene';
import ChartPanel from './components/dashboard/ChartPanel';
import CameraFeed from './components/dashboard/CameraFeed';
import ControlPanel from './components/dashboard/ControlPanel';
import DiagnosticsPanel from './components/dashboard/DiagnosticsPanel';

export default function App() {
  return (
    <div className="app-layout">
      <Header />

      <div className="app-body">
        {/* Left Sidebar - Sensor Data */}
        <div className="left-sidebar">
          <SensorPanel />
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
    </div>
  );
}
