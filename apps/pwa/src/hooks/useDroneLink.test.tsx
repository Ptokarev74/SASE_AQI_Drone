// @vitest-environment jsdom
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useDroneLink } from './useDroneLink';

class MockWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;

    static instances: MockWebSocket[] = [];

    readyState = 1; // OPEN
    send = vi.fn();
    close = vi.fn();

    onopen: (() => void) | null = null;
    onclose: (() => void) | null = null;
    onmessage: ((event: { data: string }) => void) | null = null;
    onerror: ((event: Event) => void) | null = null;

    constructor() {
        MockWebSocket.instances.push(this);
        // Simulate async open
        setTimeout(() => this.onopen?.(), 0);
    }
}

describe('useDroneLink Control Semantics', () => {
    beforeEach(() => {
        MockWebSocket.instances = [];
        vi.stubGlobal('WebSocket', MockWebSocket);
    });

    afterEach(() => {
        vi.restoreAllMocks();
        vi.useRealTimers();
    });

    it('Clamps values strictly between -1.0 and 1.0', async () => {
        const { result } = renderHook(() => useDroneLink());

        // Allow WebSocket to connect
        await act(async () => { await new Promise(r => setTimeout(r, 10)); });

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
        expect(sentMsg.arm).toBe(true);

        vi.useRealTimers();
    });

    it('Zeroes all axes and sets arm:false immediately on Disarm', async () => {
        const { result } = renderHook(() => useDroneLink());

        await act(async () => { await new Promise(r => setTimeout(r, 10)); });

        const ws = MockWebSocket.instances[0];

        act(() => {
            result.current.updateAxes({ vx: 0.8, vy: 0.5, vz: 0.3, yaw: -0.7 });
            result.current.setArmed(true);
        });

        // DISARM
        act(() => {
            result.current.setArmed(false);
        });

        const lastCall = ws.send.mock.calls[ws.send.mock.calls.length - 1][0];
        const disarmMsg = JSON.parse(lastCall);

        expect(disarmMsg.vx).toBe(0);
        expect(disarmMsg.vy).toBe(0);
        expect(disarmMsg.vz).toBe(0);
        expect(disarmMsg.yaw).toBe(0);
        expect(disarmMsg.arm).toBe(false);
    });

    it('Transmits exactly at 30Hz (~33ms intervals) while armed', async () => {
        const { result } = renderHook(() => useDroneLink());

        await act(async () => { await new Promise(r => setTimeout(r, 10)); });

        const ws = MockWebSocket.instances[0];

        vi.useFakeTimers();

        act(() => {
            result.current.setArmed(true);
        });

        act(() => {
            vi.advanceTimersByTime(1000);
        });

        // 1000ms / 33ms = ~30 ticks
        const count = ws.send.mock.calls.length;
        expect(count).toBeGreaterThanOrEqual(28);
        expect(count).toBeLessThanOrEqual(32);

        vi.useRealTimers();
    });

    it('Forces disarm and stops loop on WebSocket disconnect', async () => {
        const { result } = renderHook(() => useDroneLink());

        await act(async () => { await new Promise(r => setTimeout(r, 10)); });

        const ws = MockWebSocket.instances[0];

        vi.useFakeTimers();

        act(() => {
            result.current.updateAxes({ vx: 0.5, vy: 0.5, vz: 0.5, yaw: 0.5 });
            result.current.setArmed(true);
        });

        // Advance 100ms to accumulate some sends
        act(() => vi.advanceTimersByTime(100));
        const countBeforeDisconnect = ws.send.mock.calls.length;
        expect(countBeforeDisconnect).toBeGreaterThan(0);

        // Simulate WebSocket disconnect
        act(() => { ws.onclose?.(); });

        // Advance another 1000ms — no more sends should happen
        act(() => vi.advanceTimersByTime(1000));
        const countAfterDisconnect = ws.send.mock.calls.length;

        // No new sends after disconnect
        expect(countAfterDisconnect).toBe(countBeforeDisconnect);

        vi.useRealTimers();
    });
});
