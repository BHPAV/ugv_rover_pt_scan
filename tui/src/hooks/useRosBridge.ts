/**
 * useRosBridge - WebSocket connection hook for rosbridge
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import ROSLIB from 'roslib';
import type { ConnectionState, RosBridgeConfig } from '../lib/types.js';
import { DEFAULT_ROSBRIDGE_CONFIG } from '../lib/config.js';

export interface UseRosBridgeResult {
  ros: ROSLIB.Ros | null;
  connectionState: ConnectionState;
  error: string | null;
  connect: () => void;
  disconnect: () => void;
  reconnectAttempts: number;
}

export function useRosBridge(
  config: Partial<RosBridgeConfig> = {}
): UseRosBridgeResult {
  const fullConfig: RosBridgeConfig = { ...DEFAULT_ROSBRIDGE_CONFIG, ...config };

  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected');
  const [error, setError] = useState<string | null>(null);
  const [reconnectAttempts, setReconnectAttempts] = useState(0);

  const reconnectTimeoutRef = useRef<Timer | null>(null);
  const isUnmountedRef = useRef(false);

  const clearReconnectTimeout = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
  }, []);

  const connect = useCallback(() => {
    if (isUnmountedRef.current) return;

    clearReconnectTimeout();
    setConnectionState('connecting');
    setError(null);

    const rosInstance = new ROSLIB.Ros({
      url: fullConfig.url,
    });

    rosInstance.on('connection', () => {
      if (isUnmountedRef.current) return;
      setConnectionState('connected');
      setError(null);
      setReconnectAttempts(0);
    });

    rosInstance.on('error', (err) => {
      if (isUnmountedRef.current) return;
      const errorMsg = err instanceof Error ? err.message : 'Connection error';
      setError(errorMsg);
      setConnectionState('error');
    });

    rosInstance.on('close', () => {
      if (isUnmountedRef.current) return;
      setConnectionState('disconnected');

      // Auto-reconnect logic
      setReconnectAttempts((prev) => {
        const newAttempts = prev + 1;
        if (newAttempts < fullConfig.maxReconnectAttempts) {
          reconnectTimeoutRef.current = setTimeout(() => {
            connect();
          }, fullConfig.reconnectDelay);
        }
        return newAttempts;
      });
    });

    setRos(rosInstance);
  }, [fullConfig.url, fullConfig.reconnectDelay, fullConfig.maxReconnectAttempts, clearReconnectTimeout]);

  const disconnect = useCallback(() => {
    clearReconnectTimeout();
    if (ros) {
      ros.close();
      setRos(null);
    }
    setConnectionState('disconnected');
    setReconnectAttempts(0);
  }, [ros, clearReconnectTimeout]);

  // Connect on mount
  useEffect(() => {
    isUnmountedRef.current = false;
    connect();

    return () => {
      isUnmountedRef.current = true;
      clearReconnectTimeout();
      if (ros) {
        ros.close();
      }
    };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  return {
    ros,
    connectionState,
    error,
    connect,
    disconnect,
    reconnectAttempts,
  };
}
