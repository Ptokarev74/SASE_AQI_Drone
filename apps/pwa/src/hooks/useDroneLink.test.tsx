/**
 * @vitest-environment jsdom
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useDroneLink } from './useDroneLink';

class MockWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;

    readyState = 1; // OPEN
    send = vi.fn();
    close = vi.fn();
    onopen = vi.fn();
    onmessage = vi.fn();
    onerror = vi.fn();
    onclose = vi.fn();
    static instances: MockWebSocket[] = [];
    url: string;
    constructor(url: string) {
        this.url = url;
        MockWebSocket.instances.push(this);
    }
}

describe('useDroneLink Control Semantics', () => {
    beforeEach(() => {
        MockWebSocket.instances = [];
        vi.stubGlobal('WebSocket', MockWebSocket);
        // @ts-expect-error
        window.WebSocket = MockWebSocket;
    });

    afterEach(() => {
        vi.restoreAllMocks();
        vi.unstubAllGlobals();
        vi.useRealTimers();
    });

    it('Clamps values strictly between -1.0 and 1.0', async () => {
        vi.useRealTimers();
        const { result } = renderHook(() => useDroneLink());

        // Yield to the event loop securely so React 18 flushes the initial useEffect
        await new Promise(resolve => setTimeout(resolve, 10));

        const ws = MockWebSocket.instances[0];
        expect(ws).toBeDefined();

        vi.useFakeTimers();

        act(() => {
            result.current.updateAxes({ vx: 2.5, vy: -5.0, vz: 1.01, yaw: -1.0001 });
            result.current.setArmed(true);
        });

        act(() => {
            vi.advanceTimersByTime(34);
        });

        expect(ws.send).toHaveBeenCalled();
        const sentMsg = JSON.parse(ws.send.mock.calls[0][0]);

        expect(sentMsg.vx).toBe(1.0);
        expect(sentMsg.vy).toBe(-1.0);
        expect(sentMsg.vz).toBe(1.0);
        expect(sentMsg.yaw).toBe(-1.0);
    });

    it('Zeroes all axes and sets arm:false immediately on Disarm', async () => {
        vi.useRealTimers();
        const { result } = renderHook(() => useDroneLink());

        await new Promise(resolve => setTimeout(resolve, 10));

        const ws = MockWebSocket.instances[0];
        ws.send.mockClear();

        act(() => {
            result.current.updateAxes({ vx: 0.5, vy: 0.2, vz: 0.8, yaw: 0.1 });
            result.current.setArmed(true);
        });

        ws.send.mockClear();

        // Trigger Disarm
        act(() => {
            result.current.setArmed(false);
        });

        expect(ws.send).toHaveBeenCalledTimes(1);

        const disarmMsg = JSON.parse(ws.send.mock.calls[0][0]);
        expect(disarmMsg).toEqual({
            vx: 0,
            vy: 0,
            vz: 0,
            yaw: 0,
            arm: false
        });

        // Switch to fake timers to prove loop halted
        vi.useFakeTimers();
        act(() => {
            vi.advanceTimersByTime(100);
        });
        expect(ws.send).toHaveBeenCalledTimes(1);
    });

    it('Transmits exactly at 30Hz (~33ms intervals) while armed', async () => {
        vi.useRealTimers();
        const { result } = renderHook(() => useDroneLink());

        await new Promise(resolve => setTimeout(resolve, 10));

        const ws = MockWebSocket.instances[0];
        ws.send.mockClear();

        vi.useFakeTimers();

        act(() => {
            result.current.setArmed(true);
        });

        ws.send.mockClear();

        act(() => {
            vi.advanceTimersByTime(100); // exactly 3 ticks
        });

        expect(ws.send).toHaveBeenCalledTimes(3);
    });
});
