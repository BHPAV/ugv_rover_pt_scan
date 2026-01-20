/**
 * HelpOverlay - Help screen showing keyboard shortcuts
 */

import React from 'react';
import { Box, Text, useInput } from 'ink';

interface HelpOverlayProps {
  onClose: () => void;
}

export function HelpOverlay({ onClose }: HelpOverlayProps) {
  useInput((input, key) => {
    if (input === '?' || input === 'q' || key.escape) {
      onClose();
    }
  });

  return (
    <Box
      flexDirection="column"
      borderStyle="double"
      borderColor="yellow"
      paddingX={2}
      paddingY={1}
    >
      <Text bold color="yellow">UGV Rover PT - Keyboard Shortcuts</Text>

      <Box marginTop={1} flexDirection="column">
        <Text bold underline>Global</Text>
        <Box paddingLeft={2} flexDirection="column">
          <Box>
            <Text color="cyan" bold>?</Text>
            <Text color="gray">  Show/hide this help</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>q</Text>
            <Text color="gray">  Quit application</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>r</Text>
            <Text color="gray">  Reconnect to rosbridge</Text>
          </Box>
        </Box>
      </Box>

      <Box marginTop={1} flexDirection="column">
        <Text bold underline>Modes</Text>
        <Box paddingLeft={2} flexDirection="column">
          <Box>
            <Text color="cyan" bold>g</Text>
            <Text color="gray">  Enter gimbal control mode</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>m</Text>
            <Text color="gray">  Enter mission mode (future)</Text>
          </Box>
        </Box>
      </Box>

      <Box marginTop={1} flexDirection="column">
        <Text bold underline>Gimbal Control Mode</Text>
        <Box paddingLeft={2} flexDirection="column">
          <Box>
            <Text color="cyan" bold>← →</Text>
            <Text color="gray">  Yaw left/right</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>↑ ↓</Text>
            <Text color="gray">  Pitch up/down</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>Space</Text>
            <Text color="gray">  Center gimbal</Text>
          </Box>
          <Box>
            <Text color="cyan" bold>Esc</Text>
            <Text color="gray">  Exit mode</Text>
          </Box>
        </Box>
      </Box>

      <Box marginTop={2}>
        <Text color="gray" dimColor>Press ? or Esc to close</Text>
      </Box>
    </Box>
  );
}
