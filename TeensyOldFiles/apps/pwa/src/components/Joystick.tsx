import { useEffect, useRef } from 'react';
import nipplejs from 'nipplejs';

interface JoystickProps {
    id: string;
    type: 'left' | 'right'; // Left = Throttle/Yaw, Right = Pitch/Roll
    onMove: (data: { x: number; y: number }) => void;
    onEnd: () => void;
    // Mobile virtual sticks usually snap back to zero
    // except for traditional RC Throttle (Left Y) which stays where you leave it.
    // For this PWA, we make them both self-centering for safety.
}

export function Joystick({ id, type, onMove, onEnd }: JoystickProps) {
    const containerRef = useRef<HTMLDivElement>(null);
    const managerRef = useRef<nipplejs.JoystickManager | null>(null);

    useEffect(() => {
        if (!containerRef.current) return;

        managerRef.current = nipplejs.create({
            zone: containerRef.current,
            mode: 'static',
            position: { left: '50%', top: '50%' },
            color: type === 'left' ? 'cyan' : 'orange',
            size: 150,
        });

        const manager = managerRef.current;

        manager.on('move', (_, data) => {
            // NippleJS angle is mathematical: 0 is right, 90 is up.
            // distance is 0 to (size/2).
            const x = Math.cos(data.angle.radian) * (data.distance / 75.0);
            const y = Math.sin(data.angle.radian) * (data.distance / 75.0);

            onMove({ x, y });
        });

        manager.on('end', () => {
            onEnd();
        });

        return () => {
            manager.destroy();
        };
    }, [id, type, onMove, onEnd]);

    return (
        <div
            ref={containerRef}
            className={`relative w-48 h-48 bg-slate-800/50 rounded-full border-2 ${type === 'left' ? 'border-cyan-500/30' : 'border-orange-500/30'}`}
        />
    );
}
