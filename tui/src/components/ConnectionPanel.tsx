/**
 * ConnectionPanel - Shows connection status and feedback rate
 */

import React from 'react';
import { Box, Text } from 'ink';
import Spinner from 'ink-spinner';
import type { ConnectionState } from '../lib/types.js';

interface ConnectionPanelProps {
  connectionState: ConnectionState;
  feedbackRate: number;
  reconnectAttempts: number;
  error: string | null;
  rosbridgeUrl: string;
}

const STATE_COLORS: Record<ConnectionState, string> = {
  disconnected: 'red',
  connecting: 'yellow',
  connected: 'green',
  error: 'red',
};

const STATE_LABELS: Record<ConnectionState, string> = {
  disconnected: 'Disconnected',
  connecting: 'Connecting...',
  connected: 'Connected',
  error: 'Error',
};

export function ConnectionPanel({
  connectionState,
  feedbackRate,
  reconnectAttempts,
  error,
  rosbridgeUrl,
}: ConnectionPanelProps) {
  const stateColor = STATE_COLORS[connectionState];

  return (
    <Box flexDirection="column" borderStyle="round" borderColor="gray" paddingX={1}>
      <Text bold>Connection</Text>
      <Box flexDirection="column" marginTop={1}>
        <Box>
          <Text color="gray">rosbridge: </Text>
          <Text dimColor>{rosbridgeUrl}</Text>
        </Box>

        <Box>
          <Text color="gray">Status: </Text>
          {connectionState === 'connecting' && (
            <Text color={stateColor}>
              <Spinner type="dots" /> {STATE_LABELS[connectionState]}
            </Text>
          )}
          {connectionState !== 'connecting' && (
            <Text color={stateColor}>{STATE_LABELS[connectionState]}</Text>
          )}
        </Box>

        {connectionState === 'connected' && (
          <Box>
            <Text color="gray">Feedback: </Text>
            <Text color={feedbackRate > 5 ? 'green' : feedbackRate > 0 ? 'yellow' : 'red'}>
              {feedbackRate.toFixed(1)} Hz
            </Text>
          </Box>
        )}

        {reconnectAttempts > 0 && connectionState !== 'connected' && (
          <Box>
            <Text color="gray">Reconnect: </Text>
            <Text color="yellow">attempt {reconnectAttempts}</Text>
          </Box>
        )}

        {error && (
          <Box>
            <Text color="red">Error: {error}</Text>
          </Box>
        )}
      </Box>
    </Box>
  );
}
