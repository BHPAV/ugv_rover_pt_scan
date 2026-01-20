/**
 * BatteryPanel - Battery status with progress bar
 */

import React from 'react';
import { Box, Text } from 'ink';
import { COLORS } from '../lib/config.js';

interface BatteryPanelProps {
  voltage: number;
  percent: number;
}

function getBatteryColor(percent: number): string {
  if (percent <= COLORS.BATTERY_CRITICAL) return 'red';
  if (percent <= COLORS.BATTERY_LOW) return 'yellow';
  return 'green';
}

function createProgressBar(percent: number, width: number = 20): string {
  const filled = Math.round((percent / 100) * width);
  const empty = width - filled;
  return '[' + '█'.repeat(filled) + '░'.repeat(empty) + ']';
}

export function BatteryPanel({ voltage, percent }: BatteryPanelProps) {
  const color = getBatteryColor(percent);
  const progressBar = createProgressBar(percent);

  return (
    <Box flexDirection="column" borderStyle="round" borderColor="gray" paddingX={1}>
      <Text bold>Battery</Text>
      <Box flexDirection="column" marginTop={1}>
        <Box>
          <Text color="gray">Voltage: </Text>
          <Text color={color}>{voltage.toFixed(2)}V</Text>
        </Box>
        <Box>
          <Text color="gray">Level: </Text>
          <Text color={color}>{progressBar} {percent}%</Text>
        </Box>
        {percent <= COLORS.BATTERY_CRITICAL && (
          <Box marginTop={1}>
            <Text color="red" bold>
              ⚠ CRITICAL - Charge immediately!
            </Text>
          </Box>
        )}
        {percent > COLORS.BATTERY_CRITICAL && percent <= COLORS.BATTERY_LOW && (
          <Box marginTop={1}>
            <Text color="yellow">
              Low battery - consider charging
            </Text>
          </Box>
        )}
      </Box>
    </Box>
  );
}
