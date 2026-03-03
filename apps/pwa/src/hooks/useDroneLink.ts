import { useState, useEffect, useRef, useCallback } from 'react';

// The final, locked schema from PROTOCOL.md
export interface ControlCommand {
    vx: number;     // [-1.0, 1.0] Forward Pitch
    vy: number;     // [-1.0, 1.0] Right Roll/Strafe
    vz: number;     // [-1.0, 1.0] Ascend Throttle
    yaw: number;    // [-1.0, 1.0] CW Yaw Rate
    arm: boolean;
}

export interface TelemetryMessage {
    status: string;
    position: { x: number; y: number; z: number };
    gyro: { x: number; y: number; z: number };
    aqi: number | null;
    tvoc: number | null;
    eco2: number | null;
    temperature_c: number | null;
    humidity_pct: number | null;
    batt_v?: number | null;
}

export function useDroneLink() {
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [telemetry, setTelemetry] = useState<TelemetryMessage | null>(null);

    // Developer metrics
    const [txRate, setTxRate] = useState(0);
    const txCountRef = useRef(0);

    const socketRef = useRef<WebSocket | null>(null);

    // We use a ref for the joystick state so the 30Hz loop always sees the freshest values
    // without triggering React re-renders on every joystick micro-movement.
    const joystickStateRef = useRef<ControlCommand>({
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        yaw: 0.0,
        arm: false
    });

    const loopIntervalRef = useRef<number | null>(null);

    // Rate calculation exact measurement
    useEffect(() => {
        const interval = setInterval(() => {
            setTxRate(txCountRef.current);
            txCountRef.current = 0;
        }, 1000);
        return () => clearInterval(interval);
    }, []);

    const connect = useCallback(() => {
        if (socketRef.current?.readyState === WebSocket.OPEN) return;

        // Note: Replace with true exact URL when deploying off localhost
        const wssUrl = `ws://${window.location.hostname}:8765/ws?token=your_token_here`;
        const ws = new WebSocket(wssUrl);

        ws.onopen = () => {
            setIsConnected(true);
            setError(null);
            console.log("Drone WebSocket Connected");
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                setTelemetry(data);
            } catch (err) {
                console.error("Telemetry parse error", err);
            }
        };

        ws.onclose = () => {
            setIsConnected(false);
            console.log("Drone WebSocket Disconnected");
            stopControlLoop();
            setTimeout(connect, 3000); // Auto-reconnect
        };

        ws.onerror = (e) => {
            setError("WebSocket Error occurred");
            console.error("WS Error", e);
        };

        socketRef.current = ws;
    }, []);

    useEffect(() => {
        connect();
        return () => {
            stopControlLoop();
            if (socketRef.current) {
                socketRef.current.close();
            }
        };
    }, [connect]);


    // ---------- STRICT CONTROL SEMANTICS ----------

    // Safely clamp values to [-1.0, +1.0] and format to 4 decimal places
    const buildCommand = (): ControlCommand => {
        const raw = joystickStateRef.current;
        return {
            vx: Math.max(-1.0, Math.min(1.0, raw.vx)),
            vy: Math.max(-1.0, Math.min(1.0, raw.vy)),
            vz: Math.max(-1.0, Math.min(1.0, raw.vz)),
            yaw: Math.max(-1.0, Math.min(1.0, raw.yaw)),
            arm: raw.arm
        };
    };

    const _sendTick = () => {
        if (!socketRef.current || socketRef.current.readyState !== WebSocket.OPEN) return;
        const cmd = buildCommand();
        socketRef.current.send(JSON.stringify(cmd));
        txCountRef.current++;
    };

    const startControlLoop = () => {
        if (loopIntervalRef.current) return;
        joystickStateRef.current.arm = true;
        console.log("ARMING - Starting 30Hz Control Loop");
        // 33ms interval is ~30.3Hz
        loopIntervalRef.current = window.setInterval(_sendTick, 33);
    };

    const stopControlLoop = () => {
        if (loopIntervalRef.current) {
            window.clearInterval(loopIntervalRef.current);
            loopIntervalRef.current = null;
        }
    };

    // The public method to toggle arm state
    const setArmed = (shouldArm: boolean) => {
        if (shouldArm) {
            startControlLoop();
        } else {
            // STRICT DISARM SEMANTICS:
            // 1) Zero joystick state immediately
            joystickStateRef.current = { vx: 0, vy: 0, vz: 0, yaw: 0, arm: false };

            // 2) Synchronously transmit disarm packet exactly ONCE
            _sendTick();

            // 3) Halt the 30Hz loop
            stopControlLoop();
            console.log("DISARMED - Zeroed axes, sent stop packet, halted loop.");
        }
    };

    // The public method for the UI joysticks to update the target state
    const updateAxes = (axes: { vx?: number; vy?: number; vz?: number; yaw?: number }) => {
        joystickStateRef.current = {
            ...joystickStateRef.current,
            ...axes
        };
    };

    return {
        isConnected,
        error,
        telemetry,
        setArmed,
        updateAxes,
        currentCommand: joystickStateRef.current, // For the debug view
        txRate // Commands sent per second
    };
}
