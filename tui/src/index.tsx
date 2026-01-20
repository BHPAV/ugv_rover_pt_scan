#!/usr/bin/env bun
/**
 * UGV Rover PT TUI - Entry point
 *
 * A terminal-based UI for monitoring and controlling the UGV Rover PT
 * via rosbridge WebSocket connection.
 *
 * Usage:
 *   bun run src/index.tsx
 *   ROSBRIDGE_URL=ws://192.168.1.100:9090 bun run src/index.tsx
 */

import React from 'react';
import { render } from 'ink';
import { App } from './App.js';

// Parse command line arguments
const args = process.argv.slice(2);
let rosbridgeUrl: string | undefined;

for (let i = 0; i < args.length; i++) {
  const arg = args[i];
  if (arg === '--url' && args[i + 1]) {
    rosbridgeUrl = args[i + 1];
    i++;
  } else if (arg?.startsWith('--url=')) {
    rosbridgeUrl = arg.split('=')[1];
  } else if (arg === '--help' || arg === '-h') {
    console.log(`
UGV Rover PT TUI

Usage:
  bun run src/index.tsx [options]

Options:
  --url <url>    rosbridge WebSocket URL (default: ws://ubuntu:9090)
  --help, -h     Show this help message

Environment Variables:
  ROSBRIDGE_URL  Alternative way to set rosbridge URL

Keyboard Shortcuts:
  g              Enter gimbal control mode
  m              Enter mission mode (future)
  r              Reconnect to rosbridge
  ?              Show help
  q              Quit

In Gimbal Control Mode:
  Arrow keys     Move gimbal
  Space          Center gimbal
  q/Esc          Exit mode
`);
    process.exit(0);
  }
}

// Use environment variable as fallback
if (!rosbridgeUrl) {
  rosbridgeUrl = process.env.ROSBRIDGE_URL;
}

// Render the app
render(<App rosbridgeUrl={rosbridgeUrl} />);
