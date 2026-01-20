/**
 * Header - Application title and mode indicator
 */

import React from 'react';
import { Box, Text } from 'ink';
import type { AppMode } from '../lib/types.js';

interface HeaderProps {
  mode: AppMode;
  connected: boolean;
}

const MODE_LABELS: Record<AppMode, string> = {
  dashboard: 'Dashboard',
  gimbal: 'Gimbal Control',
  mission: 'Mission',
  help: 'Help',
};

export function Header({ mode, connected }: HeaderProps) {
  return (
    <Box
      borderStyle="single"
      borderColor={connected ? 'green' : 'red'}
      paddingX={1}
      justifyContent="space-between"
    >
      <Text bold color="cyan">
        UGV Rover PT
      </Text>
      <Text color="gray">
        [{MODE_LABELS[mode]}]
      </Text>
      <Text color={connected ? 'green' : 'red'}>
        {connected ? 'CONNECTED' : 'DISCONNECTED'}
      </Text>
    </Box>
  );
}
