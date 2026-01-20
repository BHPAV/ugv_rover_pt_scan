/**
 * IMUPanel - Accelerometer and Gyroscope display
 */

import React from 'react';
import { Box, Text } from 'ink';

interface IMUPanelProps {
  ax: number;
  ay: number;
  az: number;
  gx: number;
  gy: number;
  gz: number;
}

function formatValue(value: number, width: number = 7): string {
  return value.toFixed(2).padStart(width, ' ');
}

export function IMUPanel({ ax, ay, az, gx, gy, gz }: IMUPanelProps) {
  return (
    <Box flexDirection="column" borderStyle="round" borderColor="gray" paddingX={1}>
      <Text bold>IMU Data</Text>
      <Box flexDirection="row" marginTop={1} gap={2}>
        <Box flexDirection="column">
          <Text color="gray" underline>Accel (m/s²)</Text>
          <Box>
            <Text color="gray">X: </Text>
            <Text>{formatValue(ax)}</Text>
          </Box>
          <Box>
            <Text color="gray">Y: </Text>
            <Text>{formatValue(ay)}</Text>
          </Box>
          <Box>
            <Text color="gray">Z: </Text>
            <Text>{formatValue(az)}</Text>
          </Box>
        </Box>
        <Box flexDirection="column">
          <Text color="gray" underline>Gyro (°/s)</Text>
          <Box>
            <Text color="gray">X: </Text>
            <Text>{formatValue(gx)}</Text>
          </Box>
          <Box>
            <Text color="gray">Y: </Text>
            <Text>{formatValue(gy)}</Text>
          </Box>
          <Box>
            <Text color="gray">Z: </Text>
            <Text>{formatValue(gz)}</Text>
          </Box>
        </Box>
      </Box>
    </Box>
  );
}
