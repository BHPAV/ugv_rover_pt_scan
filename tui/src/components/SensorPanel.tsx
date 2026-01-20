/**
 * SensorPanel - Motor speeds and odometry display
 */

import React from 'react';
import { Box, Text } from 'ink';

interface SensorPanelProps {
  leftMotor: number;
  rightMotor: number;
  leftOdometry: number;
  rightOdometry: number;
}

function formatMotorSpeed(speed: number): string {
  const absSpeed = Math.abs(speed);
  const direction = speed > 0 ? '+' : speed < 0 ? '-' : ' ';
  return `${direction}${absSpeed.toString().padStart(3, ' ')}`;
}

export function SensorPanel({
  leftMotor,
  rightMotor,
  leftOdometry,
  rightOdometry,
}: SensorPanelProps) {
  return (
    <Box flexDirection="column" borderStyle="round" borderColor="gray" paddingX={1}>
      <Text bold>Motors & Odometry</Text>
      <Box flexDirection="column" marginTop={1}>
        <Box>
          <Text color="gray">Motor L/R: </Text>
          <Text color={leftMotor !== 0 ? 'cyan' : 'gray'}>
            {formatMotorSpeed(leftMotor)}
          </Text>
          <Text color="gray"> / </Text>
          <Text color={rightMotor !== 0 ? 'cyan' : 'gray'}>
            {formatMotorSpeed(rightMotor)}
          </Text>
        </Box>
        <Box>
          <Text color="gray">Odom L/R: </Text>
          <Text>{leftOdometry.toString().padStart(6, ' ')}</Text>
          <Text color="gray"> / </Text>
          <Text>{rightOdometry.toString().padStart(6, ' ')}</Text>
        </Box>
      </Box>
    </Box>
  );
}
