/**
 * useService - ROS service call hook
 */

import { useState, useCallback, useRef, useEffect } from 'react';
import ROSLIB from 'roslib';

export interface UseServiceOptions {
  ros: ROSLIB.Ros | null;
  serviceName: string;
  serviceType: string;
}

export interface UseServiceResult<TRequest, TResponse> {
  call: (request: TRequest) => Promise<TResponse>;
  isLoading: boolean;
  error: string | null;
  lastResponse: TResponse | null;
  lastCallTime: number | null;
}

export function useService<TRequest = Record<string, unknown>, TResponse = Record<string, unknown>>(
  options: UseServiceOptions
): UseServiceResult<TRequest, TResponse> {
  const { ros, serviceName, serviceType } = options;

  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [lastResponse, setLastResponse] = useState<TResponse | null>(null);
  const [lastCallTime, setLastCallTime] = useState<number | null>(null);

  const serviceRef = useRef<ROSLIB.Service | null>(null);

  useEffect(() => {
    if (!ros || !ros.isConnected) {
      serviceRef.current = null;
      return;
    }

    serviceRef.current = new ROSLIB.Service({
      ros,
      name: serviceName,
      serviceType,
    });

    return () => {
      serviceRef.current = null;
    };
  }, [ros, serviceName, serviceType]);

  const call = useCallback(
    async (request: TRequest): Promise<TResponse> => {
      if (!serviceRef.current) {
        throw new Error('Service not available');
      }

      setIsLoading(true);
      setError(null);

      return new Promise((resolve, reject) => {
        const rosRequest = new ROSLIB.ServiceRequest(request as Record<string, unknown>);

        serviceRef.current!.callService(
          rosRequest,
          (response: ROSLIB.ServiceResponse) => {
            const typedResponse = response as unknown as TResponse;
            setLastResponse(typedResponse);
            setLastCallTime(Date.now());
            setIsLoading(false);
            resolve(typedResponse);
          },
          (err: string) => {
            setError(err);
            setIsLoading(false);
            reject(new Error(err));
          }
        );
      });
    },
    []
  );

  return {
    call,
    isLoading,
    error,
    lastResponse,
    lastCallTime,
  };
}

/** Convenience hook for the ping service */
export function usePingService(ros: ROSLIB.Ros | null) {
  return useService<Record<string, never>, { success: boolean; message: string }>({
    ros,
    serviceName: '/ugv_base/ping',
    serviceType: 'std_srvs/srv/Trigger',
  });
}
