/**
 * GimbalControl - Interactive gimbal control mode
 */

import React, { useState, useEffect, useCallback } from 'react';
import { Box, Text, useInput } from 'ink';
import { GIMBAL } from '../lib/config.js';

interface GimbalControlProps {
  currentPan: number;
  currentTilt: number;
  onSetpoint: (yaw: number, pitch: number) => void;
  onExit: () => void;
  isConnected: boolean;
}

export function GimbalControl({
  currentPan,
  currentTilt,
  onSetpoint,
  onExit,
  isConnected,
}: GimbalControlProps) {
  const [targetYaw, setTargetYaw] = useState(0);
  const [targetPitch, setTargetPitch] = useState(0);

  // Clamp values to valid range
  const clampYaw = useCallback((yaw: number) => {
    return Math.max(GIMBAL.YAW_MIN, Math.min(GIMBAL.YAW_MAX, yaw));
  }, []);

  const clampPitch = useCallback((pitch: number) => {
    return Math.max(GIMBAL.PITCH_MIN, Math.min(GIMBAL.PITCH_MAX, pitch));
  }, []);

  // Send setpoint when target changes
  useEffect(() => {
    if (isConnected) {
      onSetpoint(targetYaw, targetPitch);
    }
  }, [targetYaw, targetPitch, isConnected, onSetpoint]);

  useInput((input, key) => {
    if (input === 'q' || key.escape) {
      onExit();
      return;
    }

    if (input === ' ') {
      // Center gimbal
      setTargetYaw(0);
      setTargetPitch(0);
      return;
    }

    if (key.leftArrow) {
      setTargetYaw((prev) => clampYaw(prev - GIMBAL.STEP));
    } else if (key.rightArrow) {
      setTargetYaw((prev) => clampYaw(prev + GIMBAL.STEP));
    } else if (key.upArrow) {
      setTargetPitch((prev) => clampPitch(prev + GIMBAL.STEP));
    } else if (key.downArrow) {
      setTargetPitch((prev) => clampPitch(prev - GIMBAL.STEP));
    }
  });

  return (
    <Box flexDirection="column" borderStyle="double" borderColor="cyan" paddingX={1}>
      <Text bold color="cyan">Gimbal Control Mode</Text>

      <Box flexDirection="column" marginTop={1}>
        <Box>
          <Text color="gray">Target Yaw:   </Text>
          <Text color="cyan" bold>{targetYaw.toFixed(1).padStart(7, ' ')}°</Text>
          <Text color="gray"> (current: {currentPan.toFixed(1)}°)</Text>
        </Box>
        <Box>
          <Text color="gray">Target Pitch: </Text>
          <Text color="cyan" bold>{targetPitch.toFixed(1).padStart(7, ' ')}°</Text>
          <Text color="gray"> (current: {currentTilt.toFixed(1)}°)</Text>
        </Box>
      </Box>

      <Box marginTop={1} flexDirection="column">
        <Text color="gray">Controls:</Text>
        <Box paddingLeft={2} flexDirection="column">
          <Text>← → Yaw left/right ({GIMBAL.STEP}° step)</Text>
          <Text>↑ ↓ Pitch up/down ({GIMBAL.STEP}° step)</Text>
          <Text>Space Center gimbal</Text>
          <Text>q Exit gimbal mode</Text>
        </Box>
      </Box>

      {!isConnected && (
        <Box marginTop={1}>
          <Text color="red">Not connected - commands not sent</Text>
        </Box>
      )}
    </Box>
  );
}
