import { Ros, Service } from "roslib";

import type { MapEditStroke, MapSummary } from "./types";

export interface SetBoolResponse {
  message: string;
  success: boolean;
}

export interface TriggerResponse {
  message: string;
  success: boolean;
}

export interface MapMutationResponse {
  message: string;
  success: boolean;
  summary?: MapSummary;
}

export interface DeleteMapResponse {
  maps: MapSummary[];
  message: string;
  selected_map_id: string;
  success: boolean;
}

export interface ApplyMapEditResponse {
  message: string;
  success: boolean;
  summary?: MapSummary;
}

export interface BluetoothDeviceCommandResponse {
  message: string;
  success: boolean;
}

export interface SetManualInputSourceResponse {
  active_source: string;
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

export function callBluetoothDeviceCommandService(
  ros: Ros,
  name: string,
  address: string,
  controllerType: string,
): Promise<BluetoothDeviceCommandResponse> {
  const service = new Service<{ address: string; controller_type: string }, BluetoothDeviceCommandResponse>({
    name,
    ros,
    serviceType: "mower_msgs/BluetoothDeviceCommand",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { address, controller_type: controllerType },
      resolve,
      (error) => reject(new Error(error)),
      20,
    );
  });
}

export function callSetManualInputSourceService(
  ros: Ros,
  name: string,
  source: string,
): Promise<SetManualInputSourceResponse> {
  const service = new Service<{ source: string }, SetManualInputSourceResponse>({
    name,
    ros,
    serviceType: "mower_msgs/SetManualInputSource",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { source },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callCreateMapService(ros: Ros, name: string, mapName: string): Promise<MapMutationResponse> {
  const service = new Service<{ name: string }, MapMutationResponse>({
    name,
    ros,
    serviceType: "mower_map/CreateMapSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { name: mapName },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callSelectMapService(ros: Ros, name: string, mapId: string): Promise<MapMutationResponse> {
  const service = new Service<{ map_id: string }, MapMutationResponse>({
    name,
    ros,
    serviceType: "mower_map/SelectMapSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { map_id: mapId },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callRenameMapService(ros: Ros, name: string, mapId: string, mapName: string): Promise<MapMutationResponse> {
  const service = new Service<{ map_id: string; name: string }, MapMutationResponse>({
    name,
    ros,
    serviceType: "mower_map/RenameMapSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { map_id: mapId, name: mapName },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callDeleteMapService(ros: Ros, name: string, mapId: string): Promise<DeleteMapResponse> {
  const service = new Service<{ map_id: string }, DeleteMapResponse>({
    name,
    ros,
    serviceType: "mower_map/DeleteMapSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { map_id: mapId },
      resolve,
      (error) => reject(new Error(error)),
      8,
    );
  });
}

export function callApplyMapEditService(ros: Ros, name: string, edit: MapEditStroke): Promise<ApplyMapEditResponse> {
  const service = new Service<{ edit: MapEditStroke }, ApplyMapEditResponse>({
    name,
    ros,
    serviceType: "mower_map/ApplyMapEditSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { edit },
      resolve,
      (error) => reject(new Error(error)),
      12,
    );
  });
}

export function callApplyRecordingEditService(ros: Ros, name: string, edit: MapEditStroke): Promise<ApplyMapEditResponse> {
  const service = new Service<{ edit: MapEditStroke }, ApplyMapEditResponse>({
    name,
    ros,
    serviceType: "mower_map/ApplyRecordingEditSrv",
  });

  return new Promise((resolve, reject) => {
    service.callService(
      { edit },
      resolve,
      (error) => reject(new Error(error)),
      12,
    );
  });
}
