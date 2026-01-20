# UGV Rover PT TUI v2 - Bun + Ink Proposal

## Overview

Replace the Python/Rich TUI with a modern TypeScript-based interface using **Bun** (fast JS runtime) and **Ink** (React for CLI). This enables component-based architecture, reactive state management, and richer interactivity.

## Why Bun + Ink?

| Feature | Current (Python/Rich) | Proposed (Bun/Ink) |
|---------|----------------------|-------------------|
| Startup time | ~2s | ~100ms |
| Hot reload | No | Yes |
| Components | Procedural | React-style JSX |
| State management | Manual dataclass | React hooks |
| Keyboard handling | Limited | Full support |
| Styling | Rich markup | Flexbox layout |
| Testing | Manual | Jest/Vitest |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Ink React App                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  <App>                                                    │  │
│  │    <Header title="UGV Rover PT" />                       │  │
│  │    <Box flexDirection="row">                              │  │
│  │      <ConnectionPanel esp32={...} camera={...} />        │  │
│  │      <SensorsPanel gimbal={...} motors={...} />          │  │
│  │    </Box>                                                 │  │
│  │    <Box flexDirection="row">                              │  │
│  │      <BatteryPanel voltage={...} />                      │  │
│  │      <IMUPanel accel={...} gyro={...} />                 │  │
│  │    </Box>                                                 │  │
│  │    <CommandBar />                                         │  │
│  │    <LogPanel messages={[...]} />                         │  │
│  │  </App>                                                   │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  WebSocket Client ←→ ROS 2 Bridge (rosbridge_server)     │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Proposed Features

### 1. Enhanced Dashboard Layout

```
┌──────────────────────────────────────────────────────────────────────────┐
│  🤖 UGV Rover PT Status Monitor                    12:34:56  │  ⚡ 78%  │
├──────────────────────────────────────────────────────────────────────────┤
│ ┌─ Connection ──────────────┐ ┌─ Gimbal ────────────────────────────────┐│
│ │ ESP32    ● Connected      │ │     ╭───────────────────╮              ││
│ │ Camera   ● Active 7.2 Hz  │ │     │    ·    ·    ·    │  Pan:  45°   ││
│ │ Feedback ● 15.0 Hz        │ │     │    ·    ◉    ·    │  Tilt: 15°   ││
│ │ Latency  12ms             │ │     │    ·    ·    ·    │              ││
│ └───────────────────────────┘ │     ╰───────────────────╯              ││
│                               └─────────────────────────────────────────┘│
│ ┌─ Battery ─────────────────┐ ┌─ Drive ─────────────────────────────────┐│
│ │ ████████████░░░░ 78%      │ │     ┌───┐         ┌───┐                ││
│ │ 12.02V (3S LiPo)          │ │     │ L │  0.00   │ R │  0.00          ││
│ │ Est. runtime: 45 min      │ │     └───┘         └───┘                ││
│ └───────────────────────────┘ │ Odom: -149 / -152                      ││
│                               └─────────────────────────────────────────┘│
│ ┌─ IMU ─────────────────────────────────────────────────────────────────┐│
│ │ Accel (g)    X: -0.65   Y:  0.03   Z:  0.98   │ Roll:   2.1°         ││
│ │ Gyro (°/s)   X:  0.02   Y:  0.15   Z: -0.08   │ Pitch: -1.3°         ││
│ └───────────────────────────────────────────────────────────────────────┘│
├──────────────────────────────────────────────────────────────────────────┤
│ > Ready                                            [?] Help  [Q] Quit   │
└──────────────────────────────────────────────────────────────────────────┘
```

### 2. Interactive Commands

| Key | Action |
|-----|--------|
| `g` | Gimbal control mode (arrow keys to move) |
| `m` | Start mission dialog |
| `c` | Camera preview (open in browser) |
| `l` | Toggle log panel |
| `r` | Reconnect to ROS |
| `?` | Show help |
| `q` | Quit |

### 3. Mission Control Panel

```
┌─ Mission Control ───────────────────────────────────────────────┐
│                                                                 │
│  Scan Parameters:                                               │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Yaw Range:    [-90°] ═══════●═══════ [90°]              │   │
│  │ Yaw Step:     [45°]                                      │   │
│  │ Pitch Angles: [0°, 30°]                                  │   │
│  │ Frames/Pose:  [2]                                        │   │
│  │ Settle Time:  [500ms]                                    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  [Enter] Start Mission    [Esc] Cancel                         │
└─────────────────────────────────────────────────────────────────┘
```

### 4. Live Mission Progress

```
┌─ Mission Progress ──────────────────────────────────────────────┐
│                                                                 │
│  Status: SCANNING                                               │
│  ████████████████░░░░░░░░░░░░░░ 53%                            │
│                                                                 │
│  Current Pose: Yaw 45° / Pitch 30°                             │
│  Frames: 12 / 24                                                │
│  Elapsed: 00:01:23                                              │
│                                                                 │
│  Recent:                                                        │
│  ✓ Captured frame_012.jpg (1280x720, 245KB)                    │
│  ✓ Captured frame_011.jpg (1280x720, 231KB)                    │
│  → Moving to next pose...                                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 5. Gimbal Joystick Mode

```
┌─ Gimbal Control ────────────────────────────────────────────────┐
│                                                                 │
│       Use arrow keys to move gimbal                             │
│                                                                 │
│                    ▲                                            │
│                  ┌───┐                                          │
│              ◄ ──│ ● │── ►       Pan:  45.0°                   │
│                  └───┘           Tilt: 15.0°                    │
│                    ▼                                            │
│                                                                 │
│  [Space] Center    [S] Save position    [Esc] Exit             │
└─────────────────────────────────────────────────────────────────┘
```

## Technical Implementation

### Project Structure

```
tui/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.tsx              # Entry point
│   ├── App.tsx                # Main app component
│   ├── components/
│   │   ├── Header.tsx
│   │   ├── ConnectionPanel.tsx
│   │   ├── BatteryPanel.tsx
│   │   ├── GimbalPanel.tsx
│   │   ├── DrivePanel.tsx
│   │   ├── IMUPanel.tsx
│   │   ├── MissionPanel.tsx
│   │   ├── LogPanel.tsx
│   │   └── CommandBar.tsx
│   ├── hooks/
│   │   ├── useRosBridge.ts    # WebSocket to rosbridge
│   │   ├── useKeyboard.ts     # Keyboard input handling
│   │   └── useInterval.ts     # Polling helper
│   ├── services/
│   │   ├── ros.ts             # ROS 2 message types
│   │   └── api.ts             # REST API (optional)
│   └── utils/
│       ├── formatters.ts      # Unit conversions
│       └── colors.ts          # Theme colors
├── scripts/
│   └── run.sh                 # Launcher
└── README.md
```

### Key Dependencies

```json
{
  "dependencies": {
    "ink": "^4.4.1",
    "ink-big-text": "^2.0.0",
    "ink-spinner": "^5.0.0",
    "ink-text-input": "^5.0.1",
    "react": "^18.2.0",
    "roslibjs": "^1.3.0",
    "ws": "^8.14.0"
  },
  "devDependencies": {
    "@types/react": "^18.2.0",
    "typescript": "^5.3.0"
  }
}
```

### ROS Bridge Integration

The TUI connects via WebSocket to `rosbridge_server` running in the container:

```typescript
// hooks/useRosBridge.ts
import { useEffect, useState } from 'react';
import ROSLIB from 'roslibjs';

export function useRosBridge(url: string) {
  const [connected, setConnected] = useState(false);
  const [feedback, setFeedback] = useState<Feedback | null>(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url });

    ros.on('connection', () => setConnected(true));
    ros.on('close', () => setConnected(false));

    const feedbackTopic = new ROSLIB.Topic({
      ros,
      name: '/ugv_base/feedback/json',
      messageType: 'std_msgs/String'
    });

    feedbackTopic.subscribe((msg) => {
      setFeedback(JSON.parse(msg.data));
    });

    return () => ros.close();
  }, [url]);

  return { connected, feedback };
}
```

### Sample Component

```tsx
// components/BatteryPanel.tsx
import React from 'react';
import { Box, Text } from 'ink';

interface Props {
  voltage: number;
  minV?: number;
  maxV?: number;
}

export function BatteryPanel({ voltage, minV = 10.0, maxV = 12.6 }: Props) {
  const percentage = Math.round(((voltage - minV) / (maxV - minV)) * 100);
  const barWidth = 20;
  const filled = Math.round((percentage / 100) * barWidth);

  const color = percentage > 50 ? 'green' : percentage > 20 ? 'yellow' : 'red';

  return (
    <Box flexDirection="column" borderStyle="round" borderColor="yellow" padding={1}>
      <Text bold>Battery</Text>
      <Box>
        <Text color={color}>{'█'.repeat(filled)}</Text>
        <Text color="gray">{'░'.repeat(barWidth - filled)}</Text>
        <Text> {percentage}%</Text>
      </Box>
      <Text>{voltage.toFixed(2)}V (3S LiPo)</Text>
    </Box>
  );
}
```

## Container Changes

Add rosbridge_server to enable WebSocket access:

```yaml
# docker/compose.yml - add to command
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
```

```dockerfile
# docker/Dockerfile - add dependency
RUN apt-get install -y ros-humble-rosbridge-server
```

## Migration Path

1. **Phase 1**: Add rosbridge to container, keep Python TUI
2. **Phase 2**: Build Ink TUI with basic panels
3. **Phase 3**: Add interactive features (gimbal control, mission dialog)
4. **Phase 4**: Deprecate Python TUI

## Running the New TUI

```bash
# From macOS (connects via WebSocket)
cd tui && bun run start

# Or via the launcher
./scripts/run_tui_ink.sh
```

## Benefits

1. **Faster startup**: Bun is significantly faster than Python
2. **Better UX**: React-style components enable richer interactions
3. **Cross-platform**: Runs on macOS without Docker
4. **Keyboard control**: Full interactive control of gimbal and missions
5. **Extensible**: Easy to add new panels and features
6. **Type-safe**: TypeScript catches errors at compile time

## Questions for Review

1. Should we add a web-based dashboard option too (same React components)?
2. Priority of interactive features (gimbal control vs mission dialog)?
3. Keep Python TUI as fallback or fully replace?
