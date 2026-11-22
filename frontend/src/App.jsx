import { useState } from 'react';
import Header from './components/dashboard/Header';
import MobileQuickBar from './components/dashboard/MobileQuickBar';
import SensorPanel from './components/dashboard/SensorPanel';
import JetsonTerminalBox from './components/dashboard/JetsonTerminalBox';
import VehicleScene from './components/3d/VehicleScene';
import ChartPanel from './components/dashboard/ChartPanel';
import CameraFeed from './components/dashboard/CameraFeed';
import ControlPanel from './components/dashboard/ControlPanel';
import DiagnosticsPanel from './components/dashboard/DiagnosticsPanel';
import ThrusterTestbed from './components/thruster-test/ThrusterTestbed';
import useVehicleStore from './store/vehicleStore';

const MOBILE_TABS = [
  { id: 'pilot', icon: '🕹️', label: 'Pilot' },
  { id: 'view3d', icon: '🌐', label: '3D & Grafik' },
  { id: 'sensor', icon: '📡', label: 'Sensor' },
  { id: 'jetson', icon: '💻', label: 'Jetson' },
];

export default function App() {
  const activePage = useVehicleStore((s) => s.activePage);
  const [mobileTab, setMobileTab] = useState('pilot');

  return (
    <div className="app-layout" data-mobile-tab={mobileTab}>
      <Header />

      {activePage === 'thruster_test' ? (
        <ThrusterTestbed />
      ) : (
        <>
          <MobileQuickBar />

          <div className="app-body">
            {/* Left Sidebar - Sensor Data & Terminal Command Box */}
            <div className="left-sidebar">
              <div className="mobile-section mobile-section-sensor">
                <SensorPanel />
              </div>
              <div className="mobile-section mobile-section-jetson">
                <JetsonTerminalBox />
              </div>
            </div>

            {/* Center - 3D Viewport + Charts */}
            <div className="center-area">
              <div className="mobile-section mobile-section-view3d">
                <VehicleScene />
                <ChartPanel />
              </div>
            </div>

            {/* Right Sidebar - Camera, Controls, Diagnostics */}
            <div className="right-sidebar">
              <div className="mobile-section mobile-section-pilot">
                <CameraFeed />
                <ControlPanel />
              </div>
              <div className="mobile-section mobile-section-sensor">
                <DiagnosticsPanel />
              </div>
            </div>
          </div>

          <nav className="mobile-only mobile-bottom-nav" aria-label="Mobile Navigation">
            {MOBILE_TABS.map((tab) => (
              <button
                key={tab.id}
                type="button"
                className={`mobile-nav-btn ${mobileTab === tab.id ? 'active' : ''}`}
                onClick={() => setMobileTab(tab.id)}
              >
                <span className="mobile-nav-icon">{tab.icon}</span>
                <span className="mobile-nav-label">{tab.label}</span>
              </button>
            ))}
          </nav>
        </>
      )}
    </div>
  );
}
