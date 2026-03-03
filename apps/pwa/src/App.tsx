import { useState } from 'react';
import { useDroneLink } from './hooks/useDroneLink';
import { Joystick } from './components/Joystick';
import { TelemetryPanel } from './components/TelemetryPanel';
import { Shield, ShieldAlert, Cpu, Activity, Bug } from 'lucide-react';

function App() {
  const { isConnected, telemetry, setArmed, updateAxes, currentCommand, txRate } = useDroneLink();
  const [isArmed, setIsArmedUI] = useState(false);
  const [showDebug, setShowDebug] = useState(false);

  // Mode 2 Left Joystick: Throttle (Z) and Yaw (cw/ccw)
  const handleLeftJoystick = (data: { x: number; y: number }) => {
    updateAxes({
      yaw: data.x,  // Right -> CW (+), Left -> CCW (-)
      vz: data.y    // Up -> Ascend (+), Down -> Descend (-)
    });
  };

  // Mode 2 Right Joystick: Pitch (X) and Roll (Y)
  const handleRightJoystick = (data: { x: number; y: number }) => {
    updateAxes({
      vy: data.x,   // Right -> Roll Right (+), Left -> Roll Left (-)
      vx: data.y    // Up -> Pitch Forward (+), Down -> Pitch Backward (-)
    });
  };

  const handleZeroAxes = () => {
    updateAxes({ vx: 0, vy: 0, vz: 0, yaw: 0 });
  };

  const handleArmToggle = () => {
    const nextState = !isArmed;
    setIsArmedUI(nextState);
    setArmed(nextState);
  };

  return (
    <div className="min-h-screen bg-slate-950 text-slate-50 flex flex-col font-sans overflow-hidden">

      {/* HEADER SECTION */}
      <header className="flex items-center justify-between p-4 bg-slate-900/80 border-b border-slate-800 backdrop-blur-md">
        <div className="flex items-center gap-3">
          <div className="relative">
            <Cpu className="w-8 h-8 text-cyan-400" />
            <span className={`absolute -bottom-1 -right-1 w-3 h-3 rounded-full border-2 border-slate-900 ${isConnected ? 'bg-emerald-500 animate-pulse' : 'bg-rose-500'}`} />
          </div>
          <div>
            <h1 className="text-xl font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">SASE AQI Drone</h1>
            <p className="text-xs text-slate-400 font-medium tracking-wide">
              {isConnected ? 'LINK ESTABLISHED' : 'LINK OFFLINE'}
            </p>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <button
            onClick={() => setShowDebug(!showDebug)}
            className={`p-2 rounded-lg border transition-all ${showDebug ? 'bg-slate-700 border-slate-600 text-cyan-400' : 'bg-slate-800/50 border-slate-700 text-slate-400 hover:text-slate-300 hover:bg-slate-700/50'}`}
            title="Toggle Debug View"
          >
            <Bug className="w-5 h-5" />
          </button>

          <button
            onClick={handleArmToggle}
            className={`
              relative overflow-hidden group px-8 py-3 rounded-xl font-bold text-lg tracking-widest transition-all duration-300
              ${isArmed
                ? 'bg-rose-500 hover:bg-rose-600 text-white shadow-[0_0_20px_rgba(244,63,94,0.4)]'
                : 'bg-emerald-500 hover:bg-emerald-600 text-slate-950 shadow-[0_0_20px_rgba(16,185,129,0.3)]'
              }
            `}
          >
            <div className="flex items-center gap-2">
              {isArmed ? <ShieldAlert className="w-6 h-6 animate-pulse" /> : <Shield className="w-6 h-6" />}
              {isArmed ? 'DISARM' : 'ARM'}
            </div>
          </button>
        </div>
      </header>

      {/* MAIN CONTENT DASHBOARD */}
      <main className="flex-1 p-4 lg:p-6 flex flex-col gap-6 relative">

        {/* TELEMETRY LAYER */}
        <section className="w-full">
          <TelemetryPanel data={telemetry} />
        </section>

        {/* FULL SCREEN CONTROLS LAYER (Overlays on absolute/fixed for real thumbs) */}
        <section className="flex-1 flex items-center justify-between px-8 pb-8 mt-12 w-full max-w-5xl mx-auto opacity-80 hover:opacity-100 transition-opacity">

          <div className="flex flex-col items-center gap-4">
            <div className="text-cyan-400/80 font-bold tracking-widest text-sm uppercase">Throttle / Yaw</div>
            <Joystick
              id="left-stick"
              type="left"
              onMove={handleLeftJoystick}
              onEnd={handleZeroAxes}
            />
          </div>

          {/* Optional center visualization or camera feed would go here */}
          <div className="hidden md:flex flex-col items-center justify-center pointer-events-none opacity-20">
            <Activity className="w-32 h-32 text-cyan-500" />
          </div>

          <div className="flex flex-col items-center gap-4">
            <div className="text-orange-400/80 font-bold tracking-widest text-sm uppercase">Pitch / Roll</div>
            <Joystick
              id="right-stick"
              type="right"
              onMove={handleRightJoystick}
              onEnd={handleZeroAxes}
            />
          </div>

        </section>

      </main>

      {/* DEV DEBUG OVERLAY */}
      {showDebug && (
        <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2 bg-slate-900/95 border border-slate-700 rounded-xl p-4 text-xs font-mono text-slate-300 shadow-2xl backdrop-blur-md z-50 min-w-[300px]">
          <div className="flex justify-between items-center mb-3 border-b border-slate-700 pb-2">
            <span className="font-bold text-cyan-400">PWA DEV CONSOLE</span>
            <span className={`px-2 py-0.5 rounded ${txRate >= 28 ? 'bg-emerald-500/20 text-emerald-400' : 'bg-rose-500/20 text-rose-400'}`}>
              TX RATE: {txRate} Hz
            </span>
          </div>
          <div className="grid grid-cols-2 gap-x-6 gap-y-2">
            <div className="flex justify-between"><span>Pitch (vx):</span> <span className={currentCommand.vx !== 0 ? 'text-white font-bold' : ''}>{currentCommand.vx.toFixed(4)}</span></div>
            <div className="flex justify-between"><span>Yaw (yaw):</span> <span className={currentCommand.yaw !== 0 ? 'text-white font-bold' : ''}>{currentCommand.yaw.toFixed(4)}</span></div>
            <div className="flex justify-between"><span>Roll (vy):</span> <span className={currentCommand.vy !== 0 ? 'text-white font-bold' : ''}>{currentCommand.vy.toFixed(4)}</span></div>
            <div className="flex justify-between"><span>Arm:</span> <span className={currentCommand.arm ? 'text-rose-400 font-bold' : 'text-slate-500'}>{currentCommand.arm.toString().toUpperCase()}</span></div>
            <div className="flex justify-between"><span>Ascend (vz):</span> <span className={currentCommand.vz !== 0 ? 'text-white font-bold' : ''}>{currentCommand.vz.toFixed(4)}</span></div>
          </div>
          <div className="mt-3 text-slate-500 pt-2 border-t border-slate-800 text-[10px] leading-tight text-center">
            Mode 2 | Auto-Center Disabled | Strict 30Hz Loop
          </div>
        </div>
      )}

    </div>
  );
}

export default App;
