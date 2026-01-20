/**
 * GimbalPanel - Gimbal position display with visual compass
 */

import React from 'react';
import { Box, Text } from 'ink';

interface GimbalPanelProps {
  pan: number;
  tilt: number;
}

function getCompassDirection(angle: number): string {
  // Normalize angle to 0-360
  const normalized = ((angle % 360) + 360) % 360;

  if (normalized >= 337.5 || normalized < 22.5) return 'N';
  if (normalized >= 22.5 && normalized < 67.5) return 'NE';
  if (normalized >= 67.5 && normalized < 112.5) return 'E';
  if (normalized >= 112.5 && normalized < 157.5) return 'SE';
  if (normalized >= 157.5 && normalized < 202.5) return 'S';
  if (normalized >= 202.5 && normalized < 247.5) return 'SW';
  if (normalized >= 247.5 && normalized < 292.5) return 'W';
  return 'NW';
}

function createCompassDisplay(pan: number): string[] {
  const direction = getCompassDirection(pan);
  const compassLines = [
    '     N     ',
    '  NW   NE  ',
    ' W   +   E ',
    '  SW   SE  ',
    '     S     ',
  ];

  // Highlight current direction
  return compassLines.map((line) => {
    if (line.includes(direction) && direction.length <= 2) {
      return line.replace(direction, `[${direction}]`);
    }
    return line;
  });
}

export function GimbalPanel({ pan, tilt }: GimbalPanelProps) {
  const compassLines = createCompassDisplay(pan);

  return (
    <Box flexDirection="column" borderStyle="round" borderColor="gray" paddingX={1}>
      <Text bold>Gimbal (IMU)</Text>
      <Box flexDirection="row" marginTop={1} gap={2}>
        <Box flexDirection="column">
          <Box>
            <Text color="gray">Pan:  </Text>
            <Text color="cyan">{pan.toFixed(1).padStart(7, ' ')}°</Text>
          </Box>
          <Box>
            <Text color="gray">Tilt: </Text>
            <Text color="cyan">{tilt.toFixed(1).padStart(7, ' ')}°</Text>
          </Box>
          <Box marginTop={1}>
            <Text color="gray">Dir: </Text>
            <Text color="yellow" bold>{getCompassDirection(pan)}</Text>
          </Box>
        </Box>
        <Box flexDirection="column">
          {compassLines.map((line, i) => (
            <Text key={i} color={line.includes('[') ? 'yellow' : 'gray'}>
              {line}
            </Text>
          ))}
        </Box>
      </Box>
    </Box>
  );
}
