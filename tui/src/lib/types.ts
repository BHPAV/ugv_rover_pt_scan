/**
 * UGV Rover PT TUI Type Definitions
 */

/** ESP32 T=1001 feedback message structure */
export interface UGVFeedback {
  T: number;
  L: number;         // Left motor speed
  R: number;         // Right motor speed
  ax: number;        // Accelerometer X
  ay: number;        // Accelerometer Y
  az: number;        // Accelerometer Z
  gx: number;        // Gyroscope X
  gy: number;        // Gyroscope Y
  gz: number;        // Gyroscope Z
  mx: number;        // Magnetometer X
  my: number;        // Magnetometer Y
  mz: number;        // Magnetometer Z
  odl: number;       // Left odometry
  odr: number;       // Right odometry
  v: number;         // Voltage (centivolts)
  pan: number;       // Gimbal pan angle (IMU-derived)
  tilt: number;      // Gimbal tilt angle (IMU-derived)
}

/** Parsed feedback with derived values */
export interface ParsedFeedback extends UGVFeedback {
  voltage: number;          // Voltage in volts (v / 100)
  batteryPercent: number;   // Estimated battery percentage
  timestamp: number;        // Local timestamp when received
}

/** Connection states for the TUI */
export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';

/** Application modes */
export type AppMode = 'dashboard' | 'gimbal' | 'mission' | 'help';

/** Gimbal setpoint message (geometry_msgs/Vector3) */
export interface GimbalSetpoint {
  x: number;  // Yaw angle in degrees
  y: number;  // Pitch angle in degrees
  z: number;  // Unused
}

/** Ping service response */
export interface PingResponse {
  success: boolean;
  message: string;
}

/** ROS topic message with metadata */
export interface TopicMessage<T> {
  data: T;
  timestamp: number;
  rate: number;  // Messages per second
}

/** Configuration for rosbridge connection */
export interface RosBridgeConfig {
  url: string;
  reconnectDelay: number;
  maxReconnectAttempts: number;
}
