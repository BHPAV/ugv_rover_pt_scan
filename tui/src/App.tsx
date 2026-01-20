/**
 * App - Main application component
 */

import React, { useState, useCallback, useMemo } from 'react';
import { Box, Text, useInput, useApp } from 'ink';
import {
  Header,
  ConnectionPanel,
  BatteryPanel,
  SensorPanel,
  GimbalPanel,
  IMUPanel,
  GimbalControl,
  HelpOverlay,
} from './components/index.js';
import { useRosBridge, useTopic, usePublisher } from './hooks/index.js';
import { DEFAULT_ROSBRIDGE_CONFIG, TOPICS, voltageToBatteryPercent } from './lib/config.js';
import type { AppMode, UGVFeedback, ParsedFeedback } from './lib/types.js';

interface AppProps {
  rosbridgeUrl?: string;
}

export function App({ rosbridgeUrl }: AppProps) {
  const { exit } = useApp();
  const [mode, setMode] = useState<AppMode>('dashboard');

  // ROS connection
  const url = rosbridgeUrl ?? DEFAULT_ROSBRIDGE_CONFIG.url;
  const {
    ros,
    connectionState,
    error,
    connect,
    reconnectAttempts,
  } = useRosBridge({ url });

  // Subscribe to feedback topic
  const transformFeedback = useCallback((raw: unknown): ParsedFeedback => {
    // Parse the JSON string if needed
    let data: UGVFeedback;
    if (typeof raw === 'object' && raw !== null && 'data' in raw) {
      // std_msgs/String wrapper
      const msgData = (raw as { data: string }).data;
      data = JSON.parse(msgData) as UGVFeedback;
    } else {
      data = raw as UGVFeedback;
    }

    const voltage = data.v / 100;
    return {
      ...data,
      voltage,
      batteryPercent: voltageToBatteryPercent(voltage),
      timestamp: Date.now(),
    };
  }, []);

  const { message: feedbackMsg } = useTopic<ParsedFeedback>({
    ros,
    topicName: TOPICS.FEEDBACK,
    messageType: 'std_msgs/msg/String',
    throttleMs: 100,
    transform: transformFeedback,
  });

  // Gimbal publisher
  const { publish: publishGimbal, isReady: gimbalPublisherReady } = usePublisher({
    ros,
    topicName: TOPICS.GIMBAL_SETPOINT,
    messageType: 'geometry_msgs/msg/Vector3',
  });

  const handleGimbalSetpoint = useCallback(
    (yaw: number, pitch: number) => {
      if (gimbalPublisherReady) {
        publishGimbal({ x: yaw, y: pitch, z: 0.0 });
      }
    },
    [gimbalPublisherReady, publishGimbal]
  );

  // Current feedback data with defaults
  const feedback: ParsedFeedback = useMemo(() => {
    if (feedbackMsg?.data) {
      return feedbackMsg.data;
    }
    return {
      T: 0,
      L: 0,
      R: 0,
      ax: 0,
      ay: 0,
      az: 0,
      gx: 0,
      gy: 0,
      gz: 0,
      mx: 0,
      my: 0,
      mz: 0,
      odl: 0,
      odr: 0,
      v: 0,
      pan: 0,
      tilt: 0,
      voltage: 0,
      batteryPercent: 0,
      timestamp: 0,
    };
  }, [feedbackMsg]);

  const feedbackRate = feedbackMsg?.rate ?? 0;

  // Global keyboard input
  useInput((input, key) => {
    // Don't process global keys when in special modes
    if (mode === 'gimbal' || mode === 'help') {
      return;
    }

    if (input === 'q') {
      exit();
    } else if (input === '?') {
      setMode('help');
    } else if (input === 'g') {
      setMode('gimbal');
    } else if (input === 'r') {
      connect();
    } else if (input === 'm') {
      // Future: mission mode
      setMode('mission');
    }
  });

  const isConnected = connectionState === 'connected';

  // Render help overlay
  if (mode === 'help') {
    return (
      <Box flexDirection="column">
        <Header mode={mode} connected={isConnected} />
        <HelpOverlay onClose={() => setMode('dashboard')} />
      </Box>
    );
  }

  // Render gimbal control mode
  if (mode === 'gimbal') {
    return (
      <Box flexDirection="column">
        <Header mode={mode} connected={isConnected} />
        <GimbalControl
          currentPan={feedback.pan}
          currentTilt={feedback.tilt}
          onSetpoint={handleGimbalSetpoint}
          onExit={() => setMode('dashboard')}
          isConnected={isConnected}
        />
      </Box>
    );
  }

  // Render mission mode placeholder
  if (mode === 'mission') {
    return (
      <Box flexDirection="column">
        <Header mode={mode} connected={isConnected} />
        <Box borderStyle="round" borderColor="gray" paddingX={1}>
          <Text color="yellow">Mission mode - coming soon</Text>
        </Box>
        <Box marginTop={1}>
          <Text color="gray">Press q to return to dashboard</Text>
        </Box>
      </Box>
    );
  }

  // Render main dashboard
  return (
    <Box flexDirection="column">
      <Header mode={mode} connected={isConnected} />

      <Box flexDirection="row" marginTop={1}>
        {/* Left column */}
        <Box flexDirection="column" width="50%">
          <ConnectionPanel
            connectionState={connectionState}
            feedbackRate={feedbackRate}
            reconnectAttempts={reconnectAttempts}
            error={error}
            rosbridgeUrl={url}
          />
          <BatteryPanel
            voltage={feedback.voltage}
            percent={feedback.batteryPercent}
          />
          <SensorPanel
            leftMotor={feedback.L}
            rightMotor={feedback.R}
            leftOdometry={feedback.odl}
            rightOdometry={feedback.odr}
          />
        </Box>

        {/* Right column */}
        <Box flexDirection="column" width="50%">
          <GimbalPanel pan={feedback.pan} tilt={feedback.tilt} />
          <IMUPanel
            ax={feedback.ax}
            ay={feedback.ay}
            az={feedback.az}
            gx={feedback.gx}
            gy={feedback.gy}
            gz={feedback.gz}
          />
        </Box>
      </Box>

      {/* Footer with shortcuts */}
      <Box marginTop={1}>
        <Text color="gray">
          [g] Gimbal  [m] Mission  [r] Reconnect  [?] Help  [q] Quit
        </Text>
      </Box>
    </Box>
  );
}
