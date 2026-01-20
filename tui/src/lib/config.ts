/**
 * UGV TUI Configuration
 */

import type { RosBridgeConfig } from './types.js';

/** Default rosbridge configuration */
export const DEFAULT_ROSBRIDGE_CONFIG: RosBridgeConfig = {
  url: process.env.ROSBRIDGE_URL ?? 'ws://ubuntu:9090',
  reconnectDelay: 2000,
  maxReconnectAttempts: 10,
};

/** ROS topic names */
export const TOPICS = {
  FEEDBACK: '/ugv_base/feedback/json',
  GIMBAL_SETPOINT: '/gimbal/setpoint',
  CMD_VEL: '/cmd_vel',
  IMAGE_RAW: '/image_raw',
} as const;

/** ROS service names */
export const SERVICES = {
  PING: '/ugv_base/ping',
} as const;

/** Gimbal configuration */
export const GIMBAL = {
  YAW_MIN: -180,
  YAW_MAX: 180,
  PITCH_MIN: -30,
  PITCH_MAX: 90,
  STEP: 5,  // Degrees per keypress
} as const;

/** Battery voltage thresholds for 3S LiPo */
export const BATTERY = {
  MIN_VOLTAGE: 9.9,    // 3.3V per cell - critical
  LOW_VOLTAGE: 10.8,   // 3.6V per cell - warning
  MAX_VOLTAGE: 12.6,   // 4.2V per cell - fully charged
} as const;

/** Calculate battery percentage from voltage */
export function voltageToBatteryPercent(voltage: number): number {
  const range = BATTERY.MAX_VOLTAGE - BATTERY.MIN_VOLTAGE;
  const percent = ((voltage - BATTERY.MIN_VOLTAGE) / range) * 100;
  return Math.max(0, Math.min(100, Math.round(percent)));
}

/** Color thresholds */
export const COLORS = {
  BATTERY_CRITICAL: 20,
  BATTERY_LOW: 40,
} as const;
