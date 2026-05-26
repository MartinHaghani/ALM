import { Ros, Service } from "roslib";

export interface SetBoolResponse {
  message: string;
  success: boolean;
}

export interface TriggerResponse {
  message: string;
  success: boolean;
}

export function callSetBoolService(ros: Ros, name: string, data: boolean): Promise<SetBoolResponse> {
  const service = new Service<{ data: boolean }, SetBoolResponse>({
    name,
    ros,
    serviceType: "std_srvs/SetBool",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { data },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callTriggerService(ros: Ros, name: string): Promise<TriggerResponse> {
  const service = new Service<Record<string, never>, TriggerResponse>({
    name,
    ros,
    serviceType: "std_srvs/Trigger",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      {},
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}
