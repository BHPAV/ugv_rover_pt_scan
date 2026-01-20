/**
 * useTopic - ROS topic subscription hook
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import type { TopicMessage } from '../lib/types.js';

export interface UseTopicOptions<T> {
  ros: ROSLIB.Ros | null;
  topicName: string;
  messageType: string;
  throttleMs?: number;
  transform?: (raw: unknown) => T;
}

export interface UseTopicResult<T> {
  message: TopicMessage<T> | null;
  isSubscribed: boolean;
  messageCount: number;
}

export function useTopic<T>(options: UseTopicOptions<T>): UseTopicResult<T> {
  const {
    ros,
    topicName,
    messageType,
    throttleMs = 100,
    transform,
  } = options;

  const [message, setMessage] = useState<TopicMessage<T> | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [messageCount, setMessageCount] = useState(0);

  const topicRef = useRef<ROSLIB.Topic | null>(null);
  const messageTimestampsRef = useRef<number[]>([]);
  const lastUpdateRef = useRef<number>(0);

  // Calculate message rate based on recent messages
  const calculateRate = useCallback((): number => {
    const now = Date.now();
    const recentWindow = 2000; // 2 second window
    const timestamps = messageTimestampsRef.current.filter(
      (t) => now - t < recentWindow
    );
    messageTimestampsRef.current = timestamps;

    if (timestamps.length < 2) return 0;
    const duration = (timestamps[timestamps.length - 1]! - timestamps[0]!) / 1000;
    return duration > 0 ? (timestamps.length - 1) / duration : 0;
  }, []);

  useEffect(() => {
    if (!ros || !ros.isConnected) {
      setIsSubscribed(false);
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
      throttle_rate: throttleMs,
    });

    const handleMessage = (rawMessage: ROSLIB.Message) => {
      const now = Date.now();

      // Throttle state updates
      if (now - lastUpdateRef.current < throttleMs) {
        return;
      }
      lastUpdateRef.current = now;

      messageTimestampsRef.current.push(now);
      // Keep only last 100 timestamps
      if (messageTimestampsRef.current.length > 100) {
        messageTimestampsRef.current = messageTimestampsRef.current.slice(-50);
      }

      const data = transform
        ? transform(rawMessage)
        : (rawMessage as unknown as T);

      setMessage({
        data,
        timestamp: now,
        rate: calculateRate(),
      });
      setMessageCount((prev) => prev + 1);
    };

    topic.subscribe(handleMessage);
    topicRef.current = topic;
    setIsSubscribed(true);

    return () => {
      topic.unsubscribe();
      topicRef.current = null;
      setIsSubscribed(false);
    };
  }, [ros, topicName, messageType, throttleMs, transform, calculateRate]);

  return {
    message,
    isSubscribed,
    messageCount,
  };
}

/** Hook for publishing to a ROS topic */
export interface UsePublisherOptions {
  ros: ROSLIB.Ros | null;
  topicName: string;
  messageType: string;
}

export interface UsePublisherResult {
  publish: (message: Record<string, unknown>) => void;
  isReady: boolean;
}

export function usePublisher(options: UsePublisherOptions): UsePublisherResult {
  const { ros, topicName, messageType } = options;

  const topicRef = useRef<ROSLIB.Topic | null>(null);
  const [isReady, setIsReady] = useState(false);

  useEffect(() => {
    if (!ros || !ros.isConnected) {
      setIsReady(false);
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
    });

    topicRef.current = topic;
    setIsReady(true);

    return () => {
      topicRef.current = null;
      setIsReady(false);
    };
  }, [ros, topicName, messageType]);

  const publish = useCallback((message: Record<string, unknown>) => {
    if (topicRef.current) {
      const rosMessage = new ROSLIB.Message(message);
      topicRef.current.publish(rosMessage);
    }
  }, []);

  return { publish, isReady };
}
