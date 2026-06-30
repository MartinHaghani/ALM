// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/PolygonRosConverter.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

// Rosbag for reading/writing the map to a file
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Include Messages
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "mower_map/MapArea.h"

// Include Service Messages
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/ApplyMapEditSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/CreateMapSrv.h"
#include "mower_map/DeleteMapSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMapCatalogSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/MapSummary.h"
#include "mower_map/RenameMapSrv.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_map/SelectMapSrv.h"
#include "mower_map/map_edit_geometry.h"

// JSON for map storage
#include <algorithm>
#include <chrono>
#include <cfloat>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <string>
#include <vector>
using json = nlohmann::ordered_json;

// Monitoring
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "xbot_msgs/MapSize.h"

// RPC
#include "xbot_rpc/provider.h"

const std::string MAP_FILE = "map.json";
const std::string LEGACY_MAP_FILE = "map.bag";
const std::filesystem::path MAPS_DIR = "maps";
const std::filesystem::path MAP_INDEX_FILE = MAPS_DIR / "index.json";
const std::string MAP_DATA_FILE = "map.json";
const std::string MAP_METADATA_FILE = "metadata.json";
const std::string DEFAULT_MAP_NAME = "Default Map";
const std::string LEGACY_MAP_ID = "legacy-current";

// Forward declarations
void saveMapToFile();
void buildMap();
void publishMapCatalog();

// Struct definitions for JSON serialization
struct Point {
  double x;
  double y;
};

typedef std::vector<Point> Polygon;

struct MapArea {
  std::string id;
  std::string name;
  std::string type;
  bool active;
  Polygon outline;
};

struct DockingStation {
  std::string id;
  std::string name;
  bool active;
  Point position;
  double heading;
};

struct MapData {
  std::vector<MapArea> areas;
  std::vector<DockingStation> docking_stations;

  std::vector<MapArea> getMowingAreas() {
    std::vector<MapArea> result;
    for (const auto& area : areas) {
      if (area.type == "mow") result.push_back(area);
    }
    return result;
  }

  void clear() {
    areas.clear();
    docking_stations.clear();
  }

  std::string toJsonString() const;
};

struct MapMetadata {
  std::string id;
  std::string name;
  std::string created_at;
  std::string updated_at;
  std::string map_hash;
  double datum_lat = 0.0;
  double datum_lon = 0.0;
  std::string datum_source = "unknown";
};

struct MapBounds {
  bool valid = false;
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double center_x = 0.0;
  double center_y = 0.0;
};

bool convertLegacyMapToData(MapData& target);

// JSON serialization macros
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y)

void to_json(json& j, const MapArea& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  properties["type"] = data.type;
  if (!data.active) properties["active"] = data.active;
  j["properties"] = properties;
  j["outline"] = data.outline;
}

void from_json(const json& j, MapArea& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.type = properties.value("type", "draft");
  data.active = properties.value("active", true);
  j.at("outline").get_to(data.outline);
}

void to_json(json& j, const DockingStation& data) {
  j["id"] = data.id;
  json properties = json::object();
  if (!data.name.empty()) properties["name"] = data.name;
  if (!data.active) properties["active"] = data.active;
  if (!properties.empty()) j["properties"] = properties;
  j["position"] = data.position;
  j["heading"] = data.heading;
}

void from_json(const json& j, DockingStation& data) {
  j.at("id").get_to(data.id);
  const auto& properties = j.value("properties", json::object());
  data.name = properties.value("name", "");
  data.active = properties.value("active", true);
  j.at("position").get_to(data.position);
  j.at("heading").get_to(data.heading);
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MapData, areas, docking_stations)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MapMetadata, id, name, created_at, updated_at, map_hash, datum_lat, datum_lon,
                                   datum_source)

std::string MapData::toJsonString() const {
  json json_data = *this;
  return json_data.dump(2);
}

std::string generateNanoId(size_t length = 32) {
  static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  thread_local std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);
  std::string id(length, '\0');
  std::generate_n(id.begin(), length, [&]() { return alphabet[dist(rng)]; });
  return id;
}

// Publishes the map as JSON string
ros::Publisher json_map_pub;

// Publishes the saved-map catalog as JSON string
ros::Publisher map_catalog_pub;

// Publishes the map as occupancy grid
ros::Publisher map_pub;

// Publishes the map as markers for rviz
ros::Publisher map_server_viz_array_pub;

// Publishes map size for heatmap generator
ros::Publisher map_size_pub;

// MapData instance - the source of truth for map data
MapData map_data;
MapMetadata selected_map_metadata;
std::vector<std::string> catalog_map_ids;
std::string selected_map_id;
double configured_datum_lat = 0.0;
double configured_datum_lon = 0.0;
std::string configured_datum_source = "unknown";

bool show_fake_obstacle = false;
geometry_msgs::Pose fake_obstacle_pose;

// The grid map. This is built from the polygons loaded from the file.
grid_map::GridMap map;

std::string nowIsoUtc() {
  std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  std::ostringstream stream;
  stream << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string trimMapName(const std::string& name) {
  const auto first = name.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = name.find_last_not_of(" \t\r\n");
  return name.substr(first, last - first + 1);
}

std::filesystem::path mapDirectory(const std::string& id) {
  return MAPS_DIR / id;
}

std::filesystem::path mapDataPath(const std::string& id) {
  return mapDirectory(id) / MAP_DATA_FILE;
}

std::filesystem::path mapMetadataPath(const std::string& id) {
  return mapDirectory(id) / MAP_METADATA_FILE;
}

bool hasCatalogId(const std::string& id) {
  return std::find(catalog_map_ids.begin(), catalog_map_ids.end(), id) != catalog_map_ids.end();
}

bool writeStringAtomic(const std::filesystem::path& path, const std::string& contents) {
  try {
    const auto parent = path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }

    std::filesystem::path temp_path = path;
    temp_path += "." + generateNanoId(8) + ".tmp";

    {
      std::ofstream file(temp_path, std::ios::out | std::ios::trunc);
      if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open temp file for writing: " << temp_path);
        return false;
      }
      file << contents;
      file.close();
      if (!file) {
        ROS_ERROR_STREAM("Failed while writing temp file: " << temp_path);
        std::filesystem::remove(temp_path);
        return false;
      }
    }

    std::filesystem::rename(temp_path, path);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Atomic write failed for " << path << ": " << e.what());
    return false;
  }
}

bool writeJsonAtomic(const std::filesystem::path& path, const json& value) {
  return writeStringAtomic(path, value.dump(2));
}

bool readJsonFile(const std::filesystem::path& path, json& value) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }
  file >> value;
  return true;
}

std::string stableHash(const std::string& input) {
  uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : input) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
  std::ostringstream stream;
  stream << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

std::string mapHash(const MapData& data) {
  return stableHash(data.toJsonString());
}

MapMetadata makeDefaultMetadata(const std::string& id, const std::string& requested_name) {
  const std::string now = nowIsoUtc();
  MapMetadata metadata;
  metadata.id = id;
  metadata.name = trimMapName(requested_name).empty() ? DEFAULT_MAP_NAME : trimMapName(requested_name);
  metadata.created_at = now;
  metadata.updated_at = now;
  metadata.map_hash = mapHash(MapData{});
  metadata.datum_lat = configured_datum_lat;
  metadata.datum_lon = configured_datum_lon;
  metadata.datum_source = configured_datum_source;
  return metadata;
}

bool readMapDataFromPath(const std::filesystem::path& path, MapData& target, std::string* error = nullptr) {
  try {
    json loaded_data;
    if (!readJsonFile(path, loaded_data)) {
      if (error) *error = "could not open " + path.string();
      return false;
    }
    target = loaded_data;
    return true;
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
}

bool saveMapDataToPath(const std::filesystem::path& path, const MapData& data) {
  return writeStringAtomic(path, data.toJsonString());
}

MapMetadata readMapMetadata(const std::string& id) {
  MapMetadata metadata = makeDefaultMetadata(id, id == LEGACY_MAP_ID ? "Legacy Current" : DEFAULT_MAP_NAME);
  try {
    json loaded_metadata;
    if (readJsonFile(mapMetadataPath(id), loaded_metadata)) {
      metadata = loaded_metadata;
      if (metadata.id.empty()) metadata.id = id;
      if (metadata.name.empty()) metadata.name = DEFAULT_MAP_NAME;
      if (metadata.created_at.empty()) metadata.created_at = nowIsoUtc();
      if (metadata.updated_at.empty()) metadata.updated_at = metadata.created_at;
      if (metadata.datum_source.empty()) metadata.datum_source = "unknown";
    }
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Failed to read metadata for map " << id << ": " << e.what());
  }
  return metadata;
}

bool saveMapMetadata(const MapMetadata& metadata) {
  json metadata_json = metadata;
  return writeJsonAtomic(mapMetadataPath(metadata.id), metadata_json);
}

bool saveCatalogIndex() {
  json index = json::object();
  index["selected_map_id"] = selected_map_id;
  index["maps"] = json::array();
  for (const auto& id : catalog_map_ids) {
    index["maps"].push_back({{"id", id}});
  }
  return writeJsonAtomic(MAP_INDEX_FILE, index);
}

MapBounds computeBounds(const MapData& data) {
  MapBounds bounds;
  bounds.min_x = DBL_MAX;
  bounds.min_y = DBL_MAX;
  bounds.max_x = -DBL_MAX;
  bounds.max_y = -DBL_MAX;

  for (const auto& area : data.areas) {
    if (!area.active) continue;
    if (area.type != "mow" && area.type != "nav" && area.type != "obstacle") continue;
    for (const auto& point : area.outline) {
      bounds.min_x = std::min(bounds.min_x, point.x);
      bounds.min_y = std::min(bounds.min_y, point.y);
      bounds.max_x = std::max(bounds.max_x, point.x);
      bounds.max_y = std::max(bounds.max_y, point.y);
      bounds.valid = true;
    }
  }

  if (!bounds.valid) {
    bounds.min_x = bounds.min_y = bounds.max_x = bounds.max_y = bounds.center_x = bounds.center_y = 0.0;
    return bounds;
  }

  bounds.center_x = (bounds.min_x + bounds.max_x) / 2.0;
  bounds.center_y = (bounds.min_y + bounds.max_y) / 2.0;
  return bounds;
}

mower_map::MapSummary buildMapSummary(const std::string& id) {
  const bool is_selected = id == selected_map_id;
  MapMetadata metadata = is_selected ? selected_map_metadata : readMapMetadata(id);
  MapData summary_data;
  if (is_selected) {
    summary_data = map_data;
  } else {
    std::string error;
    if (!readMapDataFromPath(mapDataPath(id), summary_data, &error)) {
      ROS_WARN_STREAM("Failed to read map data for summary " << id << ": " << error);
      summary_data.clear();
    }
  }

  mower_map::MapSummary summary;
  summary.id = id;
  summary.name = metadata.name.empty() ? id : metadata.name;
  summary.selected = is_selected;
  summary.created_at = metadata.created_at;
  summary.updated_at = metadata.updated_at;
  summary.map_hash = metadata.map_hash.empty() ? mapHash(summary_data) : metadata.map_hash;
  summary.area_count = summary_data.areas.size();
  summary.mowing_area_count = 0;
  summary.navigation_area_count = 0;
  summary.obstacle_count = 0;
  for (const auto& area : summary_data.areas) {
    if (area.type == "mow") {
      summary.mowing_area_count++;
    } else if (area.type == "nav") {
      summary.navigation_area_count++;
    } else if (area.type == "obstacle") {
      summary.obstacle_count++;
    }
  }
  summary.has_docking_station = !summary_data.docking_stations.empty();
  const MapBounds bounds = computeBounds(summary_data);
  summary.bounds_valid = bounds.valid;
  summary.min_x = bounds.min_x;
  summary.min_y = bounds.min_y;
  summary.max_x = bounds.max_x;
  summary.max_y = bounds.max_y;
  summary.center_x = bounds.center_x;
  summary.center_y = bounds.center_y;
  summary.datum_lat = metadata.datum_lat;
  summary.datum_lon = metadata.datum_lon;
  summary.datum_source = metadata.datum_source;
  return summary;
}

std::vector<mower_map::MapSummary> buildMapSummaries() {
  std::vector<mower_map::MapSummary> summaries;
  summaries.reserve(catalog_map_ids.size());
  for (const auto& id : catalog_map_ids) {
    summaries.push_back(buildMapSummary(id));
  }
  return summaries;
}

json mapSummaryToJson(const mower_map::MapSummary& summary) {
  return {{"id", summary.id},
          {"name", summary.name},
          {"selected", summary.selected},
          {"created_at", summary.created_at},
          {"updated_at", summary.updated_at},
          {"map_hash", summary.map_hash},
          {"area_count", summary.area_count},
          {"mowing_area_count", summary.mowing_area_count},
          {"navigation_area_count", summary.navigation_area_count},
          {"obstacle_count", summary.obstacle_count},
          {"has_docking_station", summary.has_docking_station},
          {"bounds_valid", summary.bounds_valid},
          {"min_x", summary.min_x},
          {"min_y", summary.min_y},
          {"max_x", summary.max_x},
          {"max_y", summary.max_y},
          {"center_x", summary.center_x},
          {"center_y", summary.center_y},
          {"datum_lat", summary.datum_lat},
          {"datum_lon", summary.datum_lon},
          {"datum_source", summary.datum_source}};
}

void publishMapCatalog() {
  if (!map_catalog_pub) {
    return;
  }

  json catalog = json::object();
  catalog["selected_map_id"] = selected_map_id;
  catalog["maps"] = json::array();
  for (const auto& summary : buildMapSummaries()) {
    catalog["maps"].push_back(mapSummaryToJson(summary));
  }

  std_msgs::String message;
  message.data = catalog.dump(2);
  map_catalog_pub.publish(message);
}

void refreshSelectedMetadataHash() {
  selected_map_metadata.id = selected_map_id;
  selected_map_metadata.map_hash = mapHash(map_data);
  selected_map_metadata.updated_at = nowIsoUtc();
  if (selected_map_metadata.name.empty()) selected_map_metadata.name = DEFAULT_MAP_NAME;
  if (selected_map_metadata.created_at.empty()) selected_map_metadata.created_at = selected_map_metadata.updated_at;
  if (selected_map_metadata.datum_source.empty()) selected_map_metadata.datum_source = configured_datum_source;
}

bool syncCompatibilityMap() {
  return saveMapDataToPath(MAP_FILE, map_data);
}

bool saveSelectedMapFiles() {
  if (selected_map_id.empty()) {
    ROS_ERROR_STREAM("Cannot save map because no selected map id is set");
    return false;
  }

  refreshSelectedMetadataHash();
  const bool map_saved = saveMapDataToPath(mapDataPath(selected_map_id), map_data);
  const bool metadata_saved = saveMapMetadata(selected_map_metadata);
  const bool compat_saved = syncCompatibilityMap();
  const bool index_saved = saveCatalogIndex();
  publishMapCatalog();
  return map_saved && metadata_saved && compat_saved && index_saved;
}

bool selectMapInternal(const std::string& id, std::string* message = nullptr) {
  if (!hasCatalogId(id)) {
    if (message) *message = "Unknown map id: " + id;
    return false;
  }

  MapData loaded_map;
  std::string error;
  if (!readMapDataFromPath(mapDataPath(id), loaded_map, &error)) {
    if (message) *message = "Failed to load selected map: " + error;
    ROS_ERROR_STREAM("Failed to select map " << id << ": " << error);
    return false;
  }

  selected_map_id = id;
  selected_map_metadata = readMapMetadata(id);
  selected_map_metadata.map_hash = mapHash(loaded_map);
  map_data = loaded_map;
  show_fake_obstacle = false;
  syncCompatibilityMap();
  saveMapMetadata(selected_map_metadata);
  saveCatalogIndex();
  buildMap();
  publishMapCatalog();
  if (message) *message = "Selected map: " + selected_map_metadata.name;
  return true;
}

std::string generateMapId() {
  for (int i = 0; i < 100; ++i) {
    const std::string id = generateNanoId(16);
    if (!hasCatalogId(id) && !std::filesystem::exists(mapDirectory(id))) {
      return id;
    }
  }
  return generateNanoId(32);
}

bool createMapInternal(const std::string& requested_name, mower_map::MapSummary* summary = nullptr) {
  const std::string id = generateMapId();
  MapData blank_map;
  MapMetadata metadata = makeDefaultMetadata(id, requested_name);

  catalog_map_ids.push_back(id);
  selected_map_id = id;
  selected_map_metadata = metadata;
  map_data = blank_map;
  show_fake_obstacle = false;

  if (!saveSelectedMapFiles()) {
    ROS_ERROR_STREAM("Failed to persist created map " << id);
    return false;
  }

  buildMap();
  if (summary) *summary = buildMapSummary(id);
  return true;
}

bool renameMapInternal(const std::string& id, const std::string& requested_name, std::string* message = nullptr) {
  const std::string name = trimMapName(requested_name);
  if (name.empty()) {
    if (message) *message = "Map name cannot be empty";
    return false;
  }
  if (!hasCatalogId(id)) {
    if (message) *message = "Unknown map id: " + id;
    return false;
  }

  MapMetadata metadata = id == selected_map_id ? selected_map_metadata : readMapMetadata(id);
  metadata.name = name;
  metadata.updated_at = nowIsoUtc();
  if (!saveMapMetadata(metadata)) {
    if (message) *message = "Failed to save metadata for map: " + id;
    return false;
  }
  if (id == selected_map_id) selected_map_metadata = metadata;
  publishMapCatalog();
  if (message) *message = "Renamed map: " + name;
  return true;
}

std::string newestRemainingMapId() {
  std::string newest_id;
  std::string newest_updated_at;
  for (const auto& id : catalog_map_ids) {
    const auto metadata = readMapMetadata(id);
    if (newest_id.empty() || metadata.updated_at > newest_updated_at) {
      newest_id = id;
      newest_updated_at = metadata.updated_at;
    }
  }
  return newest_id;
}

bool deleteMapInternal(const std::string& id, std::string* message = nullptr) {
  if (!hasCatalogId(id)) {
    if (message) *message = "Unknown map id: " + id;
    return false;
  }

  catalog_map_ids.erase(std::remove(catalog_map_ids.begin(), catalog_map_ids.end(), id), catalog_map_ids.end());
  try {
    std::filesystem::remove_all(mapDirectory(id));
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Failed to remove map directory " << mapDirectory(id) << ": " << e.what());
  }

  if (catalog_map_ids.empty()) {
    if (!createMapInternal(DEFAULT_MAP_NAME)) {
      if (message) *message = "Deleted map, but failed to create replacement default map";
      return false;
    }
    if (message) *message = "Deleted map and created a new default map";
    return true;
  }

  if (selected_map_id == id) {
    const std::string fallback_id = newestRemainingMapId();
    if (!selectMapInternal(fallback_id, message)) {
      map_data.clear();
      selected_map_id = fallback_id;
      selected_map_metadata = readMapMetadata(fallback_id);
      saveCatalogIndex();
      buildMap();
      publishMapCatalog();
      return false;
    }
  } else {
    saveCatalogIndex();
    publishMapCatalog();
    if (message) *message = "Deleted map";
  }

  return true;
}

void readConfiguredDatum(ros::NodeHandle& n) {
  configured_datum_source = "unknown";
  if (n.getParam("/hw/services/gps/datum_lat", configured_datum_lat) &&
      n.getParam("/hw/services/gps/datum_long", configured_datum_lon)) {
    configured_datum_source = "/hw/services/gps";
    return;
  }
  if (n.getParam("/ll/services/gps/datum_lat", configured_datum_lat) &&
      n.getParam("/ll/services/gps/datum_long", configured_datum_lon)) {
    configured_datum_source = "/ll/services/gps";
    return;
  }
  configured_datum_lat = 0.0;
  configured_datum_lon = 0.0;
  ROS_WARN_STREAM("No configured GPS datum found; map metadata datum fields will be 0,0");
}

void loadCatalogIndex() {
  catalog_map_ids.clear();
  selected_map_id.clear();

  try {
    json index;
    if (!readJsonFile(MAP_INDEX_FILE, index)) {
      return;
    }

    selected_map_id = index.value("selected_map_id", "");
    for (const auto& entry : index.value("maps", json::array())) {
      const std::string id = entry.value("id", "");
      if (!id.empty() && !hasCatalogId(id)) {
        catalog_map_ids.push_back(id);
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to read map catalog index: " << e.what());
    catalog_map_ids.clear();
    selected_map_id.clear();
  }
}

bool importInitialMap(const std::string& id, const std::string& name, const MapData& data) {
  catalog_map_ids.clear();
  catalog_map_ids.push_back(id);
  selected_map_id = id;
  selected_map_metadata = makeDefaultMetadata(id, name);
  map_data = data;
  return saveSelectedMapFiles();
}

void initializeCatalog(ros::NodeHandle& n) {
  readConfiguredDatum(n);
  std::filesystem::create_directories(MAPS_DIR);
  loadCatalogIndex();

  if (catalog_map_ids.empty()) {
    MapData initial_map;
    if (std::filesystem::exists(MAP_FILE)) {
      std::string error;
      if (readMapDataFromPath(MAP_FILE, initial_map, &error)) {
        ROS_INFO_STREAM("Migrating existing " << MAP_FILE << " into map catalog");
        importInitialMap(LEGACY_MAP_ID, "Legacy Current", initial_map);
      } else {
        ROS_ERROR_STREAM("Failed to migrate existing " << MAP_FILE << ": " << error);
        createMapInternal(DEFAULT_MAP_NAME);
      }
    } else if (std::filesystem::exists(LEGACY_MAP_FILE) && convertLegacyMapToData(initial_map)) {
      ROS_INFO_STREAM("Migrating legacy " << LEGACY_MAP_FILE << " into map catalog");
      importInitialMap(LEGACY_MAP_ID, "Legacy Current", initial_map);
    } else {
      createMapInternal(DEFAULT_MAP_NAME);
    }
  }

  if (!hasCatalogId(selected_map_id) && !catalog_map_ids.empty()) {
    selected_map_id = newestRemainingMapId();
  }

  std::string message;
  if (!selected_map_id.empty() && !selectMapInternal(selected_map_id, &message)) {
    ROS_ERROR_STREAM("Selected map could not be loaded; publishing a blank safe map. " << message);
    selected_map_metadata = readMapMetadata(selected_map_id);
    map_data.clear();
    syncCompatibilityMap();
    saveCatalogIndex();
  }
}

// clang-format off
xbot_rpc::RpcProvider rpc_provider("mower_map_service", {{
  RPC_METHOD("map.replace", {
    if (!params.is_array() || params.size() != 1) {
      throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Missing map parameter");
    }
    try {
      map_data = params[0];
    } catch (const std::exception& e) {
      throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Invalid map: " + std::string(e.what()));
    }
    saveMapToFile();
    ROS_INFO_STREAM("Loaded " << map_data.areas.size() << " areas via RPC and saved to file");
    buildMap();
    return "Successfully stored map (" + std::to_string(map_data.areas.size()) + " areas)";
  }),
}});
// clang-format on

/**
 * Convert a geometry_msgs::Polygon to our internal Polygon struct
 */
Polygon geometryPolygonToInternal(const geometry_msgs::Polygon& poly) {
  Polygon result;
  for (const auto& point : poly.points) {
    result.push_back({point.x, point.y});
  }
  return result;
}

/**
 * Convert our internal Polygon struct to geometry_msgs::Polygon
 */
geometry_msgs::Polygon internalPolygonToGeometry(const Polygon& poly) {
  geometry_msgs::Polygon result;
  for (const auto& point : poly) {
    geometry_msgs::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    result.points.push_back(pt);
  }
  return result;
}

/**
 * Convert a mower_map::MapArea to our internal MapArea struct
 */
MapArea mowerMapAreaToInternal(const geometry_msgs::Polygon& area, const std::string& type, const std::string& name) {
  MapArea result;
  result.id = generateNanoId();
  result.type = type;
  result.name = name;
  result.active = true;
  result.outline = geometryPolygonToInternal(area);
  return result;
}

/**
 * Convert our internal MapArea struct to mower_map::MapArea
 */
mower_map::MapArea internalMapAreaToMower(const MapArea& area) {
  mower_map::MapArea result;
  result.name = area.name;
  // Leave area empty if it is not active
  if (area.active) {
    result.area = internalPolygonToGeometry(area.outline);
  }
  return result;
}

grid_map::Polygon internalPolygonToGridMap(const Polygon& poly) {
  grid_map::Polygon result;
  for (const auto& point : poly) {
    result.addVertex(grid_map::Position(point.x, point.y));
  }
  return result;
}

/**
 * Publish map to xbot_monitoring
 */
void publishMapMonitoring() {
  xbot_msgs::MapSize map_size;
  map_size.mapWidth = map.getSize().x() * map.getResolution();
  map_size.mapHeight = map.getSize().y() * map.getResolution();
  auto mapPos = map.getPosition();
  map_size.mapCenterX = mapPos.x();
  map_size.mapCenterY = mapPos.y();
  map_size_pub.publish(map_size);

  std_msgs::String json_map;
  json_map.data = map_data.toJsonString();
  json_map_pub.publish(json_map);
}

/**
 * Publish map visualizations for rviz.
 */
void visualizeAreas() {
  auto mapPos = map.getPosition();

  visualization_msgs::MarkerArray markerArray;

  for (const auto& area : map_data.areas) {
    if (!area.active) continue;
    if (area.type != "mow" && area.type != "obstacle") continue;

    std_msgs::ColorRGBA color;
    if (area.type == "mow") {
      color.g = 1.0;
    } else if (area.type == "obstacle") {
      color.r = 1.0;
    }
    color.a = 1.0;

    grid_map::Polygon p = internalPolygonToGridMap(area.outline);
    visualization_msgs::Marker marker;
    grid_map::PolygonRosConverter::toLineMarker(p, color, 0.05, 0, marker);

    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size();
    marker.frame_locked = true;
    marker.pose.orientation.w = 1.0;

    markerArray.markers.push_back(marker);
  }

  // Visualize Docking Point
  if (!map_data.docking_stations.empty()) {
    const DockingStation& ds = map_data.docking_stations.front();
    geometry_msgs::Pose docking_pose;
    docking_pose.position.x = ds.position.x;
    docking_pose.position.y = ds.position.y;
    docking_pose.position.z = 0.0;

    double heading = ds.heading;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    docking_pose.orientation = tf2::toMsg(q);

    std_msgs::ColorRGBA color;
    color.b = 1.0;
    color.a = 1.0;
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color = color;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = docking_pose;
    ROS_INFO_STREAM("docking pose: " << docking_pose);
    marker.header.frame_id = "map";
    marker.ns = "mower_map_service";
    marker.id = markerArray.markers.size() + 1;
    marker.frame_locked = true;
    markerArray.markers.push_back(marker);
  }

  map_server_viz_array_pub.publish(markerArray);
}

/**
 * Uses the polygons stored in MapData to build the final occupancy grid.
 *
 * First, the map is marked as completely occupied. Then navigation_areas and mowing_areas are marked as free.
 *
 * Then, all obstacles are marked as occupied.
 *
 * Finally, a blur is applied to the map so that it is expensive, but not completely forbidden to drive near boundaries.
 */
void buildMap() {
  // First, calculate the size of the map by finding the min and max values for x and y.
  float minX = FLT_MAX;
  float maxX = -FLT_MAX;
  float minY = FLT_MAX;
  float maxY = -FLT_MAX;

  // loop through all areas and calculate a size where everything fits
  bool has_valid_area = false;
  for (const auto& area : map_data.areas) {
    if (!area.active) continue;
    if (area.type != "mow" && area.type != "nav" && area.type != "obstacle") continue;
    for (const auto& point : area.outline) {
      minX = std::min(minX, (float)point.x);
      maxX = std::max(maxX, (float)point.x);
      minY = std::min(minY, (float)point.y);
      maxY = std::max(maxY, (float)point.y);
      has_valid_area = true;
    }
  }

  // Enlarge the map by 1m in all directions.
  // This guarantees that even after blurring, the map has an occupied border.
  maxX += 1.0;
  minX -= 1.0;
  maxY += 1.0;
  minY -= 1.0;

  // Check, if the map was empty. If so, we'd create a huge map. Therefore we build an empty 10x10m map instead.
  if (!has_valid_area) {
    maxX = 5.0;
    minX = -5.0;
    maxY = 5.0;
    minY = -5.0;
  }

  map = grid_map::GridMap({"navigation_area"});
  map.setFrameId("map");
  grid_map::Position origin;
  origin.x() = (maxX + minX) / 2.0;
  origin.y() = (maxY + minY) / 2.0;

  ROS_INFO_STREAM("Map Position: x=" << origin.x() << ", y=" << origin.y());
  ROS_INFO_STREAM("Map Size: x=" << (maxX - minX) << ", y=" << (maxY - minY));

  map.setGeometry(grid_map::Length(maxX - minX, maxY - minY), 0.05, origin);
  map.setTimestamp(ros::Time::now().toNSec());

  map.clearAll();
  map["navigation_area"].setConstant(1.0);

  grid_map::Matrix& data = map["navigation_area"];
  for (const auto& area : map_data.areas) {
    if (!area.active) continue;

    double value;
    if (area.type == "mow" || area.type == "nav") {
      value = 0.0;
    } else if (area.type == "obstacle") {
      value = 1.0;
    } else {
      continue;
    }

    grid_map::Polygon poly = internalPolygonToGridMap(area.outline);
    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = value;
    }
  }

  if (show_fake_obstacle) {
    grid_map::Polygon poly;
    tf2::Quaternion q;
    tf2::fromMsg(fake_obstacle_pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);

    Eigen::Vector2d front(cos(yaw), sin(yaw));
    Eigen::Vector2d left(-sin(yaw), cos(yaw));
    Eigen::Vector2d obstacle_pos(fake_obstacle_pose.position.x, fake_obstacle_pose.position.y);

    {
      grid_map::Position pos = obstacle_pos + 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos + 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }

    {
      grid_map::Position pos = obstacle_pos - 0.6 * left + 0.7 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.6 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.2 * left - 0.1 * front;
      poly.addVertex(pos);
    }
    {
      grid_map::Position pos = obstacle_pos - 0.1 * left + 0.25 * front;
      poly.addVertex(pos);
    }
    for (grid_map::PolygonIterator iterator(map, poly); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      data(index[0], index[1]) = 1.0;
    }
  }

  cv::Mat cv_map;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "navigation_area", CV_8UC1, cv_map);

  cv::blur(cv_map, cv_map, cv::Size(5, 5));

  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(cv_map, "navigation_area", map);

  nav_msgs::OccupancyGrid msg;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, "navigation_area", 0.0, 1.0, msg);
  map_pub.publish(msg);

  publishMapMonitoring();
  visualizeAreas();
}

/**
 * Saves the current map data to a JSON file.
 * We don't need to save the grid map, since we can easily build it again after loading.
 */
void saveMapToFile() {
  if (saveSelectedMapFiles()) {
    ROS_INFO_STREAM("Map saved to selected catalog entry: " << selected_map_id);
  } else {
    ROS_ERROR_STREAM("Failed to save selected map: " << selected_map_id);
  }
}

/**
 * Load the map from a JSON file and build a map.
 */
void readMapFromFile() {
  std::string message;
  if (!selected_map_id.empty() && selectMapInternal(selected_map_id, &message)) {
    ROS_INFO_STREAM(message);
  } else {
    ROS_WARN_STREAM("Could not load selected map file: " << message);
  }
}

bool addMowingArea(mower_map::AddMowingAreaSrvRequest& req, mower_map::AddMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got addMowingArea call");

  map_data.areas.push_back(mowerMapAreaToInternal(req.area.area, req.isNavigationArea ? "nav" : "mow", req.area.name));
  for (const auto& obstacle : req.area.obstacles) {
    map_data.areas.push_back(mowerMapAreaToInternal(obstacle, "obstacle", ""));
  }

  saveMapToFile();
  buildMap();
  return true;
}

bool getMowingArea(mower_map::GetMowingAreaSrvRequest& req, mower_map::GetMowingAreaSrvResponse& res) {
  ROS_INFO_STREAM("Got getMowingArea call with index: " << req.index);

  auto mowing_areas = map_data.getMowingAreas();
  if (req.index >= mowing_areas.size()) {
    ROS_ERROR_STREAM("No mowing area with index: " << req.index);
    return false;
  }

  res.area = internalMapAreaToMower(mowing_areas[req.index]);

  for (const auto& area : map_data.areas) {
    if (!area.active || area.type != "obstacle") continue;
    res.area.obstacles.push_back(internalPolygonToGeometry(area.outline));
  }

  return true;
}

bool setDockingPoint(mower_map::SetDockingPointSrvRequest& req, mower_map::SetDockingPointSrvResponse& res) {
  ROS_INFO_STREAM("Setting Docking Point");

  // Convert quaternion to heading
  tf2::Quaternion q;
  tf2::fromMsg(req.docking_pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double unused1, unused2, heading;
  m.getRPY(unused1, unused2, heading);

  map_data.docking_stations.clear();
  map_data.docking_stations.push_back({.id = generateNanoId(),
                                       .name = "Docking Station",
                                       .active = true,
                                       .position = {req.docking_pose.position.x, req.docking_pose.position.y},
                                       .heading = heading});

  saveMapToFile();
  buildMap();

  return true;
}

bool getDockingPoint(mower_map::GetDockingPointSrvRequest& req, mower_map::GetDockingPointSrvResponse& res) {
  ROS_INFO_STREAM("Getting Docking Point");

  if (map_data.docking_stations.empty()) {
    return false;
  }

  const DockingStation& ds = map_data.docking_stations.front();
  res.docking_pose.position.x = ds.position.x;
  res.docking_pose.position.y = ds.position.y;
  res.docking_pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ds.heading);
  res.docking_pose.orientation = tf2::toMsg(q);

  return true;
}

bool setNavPoint(mower_map::SetNavPointSrvRequest& req, mower_map::SetNavPointSrvResponse& res) {
  ROS_INFO_STREAM("Setting Nav Point");

  fake_obstacle_pose = req.nav_pose;

  show_fake_obstacle = true;

  buildMap();

  return true;
}

bool clearNavPoint(mower_map::ClearNavPointSrvRequest& req, mower_map::ClearNavPointSrvResponse& res) {
  ROS_INFO_STREAM("Clearing Nav Point");

  if (show_fake_obstacle) {
    show_fake_obstacle = false;

    buildMap();
  }

  return true;
}

bool clearMap(mower_map::ClearMapSrvRequest& req, mower_map::ClearMapSrvResponse& res) {
  ROS_INFO_STREAM("Clearing Map");

  map_data.clear();

  saveMapToFile();
  buildMap();
  return true;
}

std::vector<MapArea>::iterator findEditableArea(const std::string& requested_id, std::string& message) {
  auto is_editable = [](const MapArea& area) {
    return area.active && (area.type == "mow" || area.type == "nav" || area.type == "obstacle");
  };

  if (!requested_id.empty()) {
    auto it = std::find_if(map_data.areas.begin(), map_data.areas.end(), [&](const MapArea& area) {
      return area.id == requested_id && is_editable(area);
    });
    if (it == map_data.areas.end()) {
      message = "No editable active area with id: " + requested_id;
    }
    return it;
  }

  auto it = std::find_if(map_data.areas.begin(), map_data.areas.end(), [](const MapArea& area) {
    return area.active && area.type == "mow";
  });
  if (it == map_data.areas.end()) {
    message = "Selected map has no mowing area to edit";
  }
  return it;
}

MapArea makeEditedAreaFromPolygon(const geometry_msgs::Polygon& polygon, const std::string& type, const std::string& name) {
  return mowerMapAreaToInternal(polygon, type, name);
}

bool validateEditPreconditions(const mower_map::MapEditStroke& edit, std::string& message) {
  if (!edit.expected_map_id.empty() && edit.expected_map_id != selected_map_id) {
    message = "Selected map changed before edit was applied";
    return false;
  }
  const std::string current_hash = mapHash(map_data);
  if (!edit.expected_map_hash.empty() && edit.expected_map_hash != current_hash) {
    message = "Selected map changed before edit was applied";
    return false;
  }
  return true;
}

bool applyMapEdit(mower_map::ApplyMapEditSrvRequest& req, mower_map::ApplyMapEditSrvResponse& res) {
  ROS_INFO_STREAM("Applying map edit operation " << static_cast<int>(req.edit.operation));

  std::string message;
  if (!validateEditPreconditions(req.edit, message)) {
    res.success = false;
    res.message = message;
    res.summary = buildMapSummary(selected_map_id);
    return true;
  }

  auto target = findEditableArea(req.edit.target_area_id, message);
  if (target == map_data.areas.end()) {
    res.success = false;
    res.message = message;
    res.summary = buildMapSummary(selected_map_id);
    return true;
  }

  geometry_msgs::Polygon target_polygon = internalPolygonToGeometry(target->outline);

  if (req.edit.operation == mower_map::MapEditStroke::REPLACE_POLYGON) {
    geometry_msgs::Polygon normalized;
    std::string error;
    if (!mower_map::edit_geometry::normalizeReplacementPolygon(req.edit.replacement_polygon, normalized, error)) {
      res.success = false;
      res.message = "Invalid replacement polygon: " + error;
      res.summary = buildMapSummary(selected_map_id);
      return true;
    }
    target->outline = geometryPolygonToInternal(normalized);
  } else if (req.edit.operation == mower_map::MapEditStroke::BRUSH_ADD) {
    auto edit_result = mower_map::edit_geometry::applyBrushAdd(target_polygon, req.edit.path, req.edit.brush_diameter_m);
    if (!edit_result.success) {
      res.success = false;
      res.message = "Brush add failed: " + edit_result.error;
      res.summary = buildMapSummary(selected_map_id);
      return true;
    }
    target->outline = geometryPolygonToInternal(edit_result.outline);
  } else if (req.edit.operation == mower_map::MapEditStroke::BRUSH_ERASE) {
    auto edit_result =
        mower_map::edit_geometry::applyBrushErase(target_polygon, req.edit.path, req.edit.brush_diameter_m);
    if (!edit_result.success) {
      res.success = false;
      res.message = "Brush erase failed: " + edit_result.error;
      res.summary = buildMapSummary(selected_map_id);
      return true;
    }

    const std::string target_type = target->type;
    const std::string target_name = target->name;
    if (target_type == "obstacle" && !edit_result.obstacles.empty()) {
      res.success = false;
      res.message = "Erase would create a hole inside an obstacle; edit the obstacle boundary instead";
      res.summary = buildMapSummary(selected_map_id);
      return true;
    }
    target->outline = geometryPolygonToInternal(edit_result.outline);
    for (const auto& extra_outline : edit_result.extra_outlines) {
      map_data.areas.push_back(makeEditedAreaFromPolygon(extra_outline, target_type, target_name));
    }
    if (target_type != "obstacle") {
      for (const auto& obstacle : edit_result.obstacles) {
        map_data.areas.push_back(makeEditedAreaFromPolygon(obstacle, "obstacle", "Eraser"));
      }
    }
  } else {
    res.success = false;
    res.message = "Unknown map edit operation";
    res.summary = buildMapSummary(selected_map_id);
    return true;
  }

  saveMapToFile();
  buildMap();
  res.success = true;
  res.message = "Map edit applied";
  res.summary = buildMapSummary(selected_map_id);
  return true;
}

bool getMapCatalog(mower_map::GetMapCatalogSrvRequest& req, mower_map::GetMapCatalogSrvResponse& res) {
  (void)req;
  res.success = true;
  res.message = "Loaded map catalog";
  res.selected_map_id = selected_map_id;
  res.maps = buildMapSummaries();
  return true;
}

bool createMap(mower_map::CreateMapSrvRequest& req, mower_map::CreateMapSrvResponse& res) {
  ROS_INFO_STREAM("Creating map: " << req.name);
  mower_map::MapSummary summary;
  if (!createMapInternal(req.name, &summary)) {
    res.success = false;
    res.message = "Failed to create map";
    return true;
  }
  res.success = true;
  res.message = "Created map: " + summary.name;
  res.summary = summary;
  return true;
}

bool selectMap(mower_map::SelectMapSrvRequest& req, mower_map::SelectMapSrvResponse& res) {
  ROS_INFO_STREAM("Selecting map: " << req.map_id);
  std::string message;
  if (!selectMapInternal(req.map_id, &message)) {
    res.success = false;
    res.message = message;
    return true;
  }
  res.success = true;
  res.message = message;
  res.summary = buildMapSummary(req.map_id);
  return true;
}

bool renameMap(mower_map::RenameMapSrvRequest& req, mower_map::RenameMapSrvResponse& res) {
  ROS_INFO_STREAM("Renaming map: " << req.map_id << " -> " << req.name);
  std::string message;
  if (!renameMapInternal(req.map_id, req.name, &message)) {
    res.success = false;
    res.message = message;
    return true;
  }
  res.success = true;
  res.message = message;
  res.summary = buildMapSummary(req.map_id);
  return true;
}

bool deleteMap(mower_map::DeleteMapSrvRequest& req, mower_map::DeleteMapSrvResponse& res) {
  ROS_WARN_STREAM("Deleting map: " << req.map_id);
  std::string message;
  if (!deleteMapInternal(req.map_id, &message)) {
    res.success = false;
    res.message = message;
    res.selected_map_id = selected_map_id;
    res.maps = buildMapSummaries();
    return true;
  }
  res.success = true;
  res.message = message;
  res.selected_map_id = selected_map_id;
  res.maps = buildMapSummaries();
  return true;
}

/**
 * Helper function to convert legacy map areas from bag file
 * @param bag The rosbag to read from
 * @param topic_name The topic name to query (e.g. "mowing_areas" or "navigation_areas")
 * @param area_type The type to assign to the converted areas (e.g. "mow" or "nav")
 */
void convertLegacyAreas(rosbag::Bag& bag, const std::string& topic_name, const std::string& area_type,
                        MapData& target) {
  rosbag::View view(bag, rosbag::TopicQuery(topic_name));
  for (rosbag::MessageInstance const m : view) {
    auto area = m.instantiate<mower_map::MapArea>();
    if (area) {
      // Convert main area
      MapArea main_area;
      main_area.id = generateNanoId();
      main_area.name = area->name;
      main_area.type = area_type;
      main_area.active = true;
      main_area.outline = geometryPolygonToInternal(area->area);
      target.areas.push_back(main_area);

      // Convert obstacles as separate areas
      for (const auto& obstacle : area->obstacles) {
        MapArea obs_area;
        obs_area.id = generateNanoId();
        obs_area.name = "";
        obs_area.type = "obstacle";
        obs_area.active = true;
        obs_area.outline = geometryPolygonToInternal(obstacle);
        target.areas.push_back(obs_area);
      }
    }
  }
}

bool convertLegacyMapToData(MapData& target) {
  // Open the legacy map file
  rosbag::Bag bag;
  try {
    bag.open(LEGACY_MAP_FILE);
  } catch (rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening legacy map file for conversion: " << e.what());
    return false;
  }

  target.clear();

  // Read mowing and navigation areas
  convertLegacyAreas(bag, "mowing_areas", "mow", target);
  convertLegacyAreas(bag, "navigation_areas", "nav", target);

  // Read docking point
  {
    rosbag::View view(bag, rosbag::TopicQuery("docking_point"));
    for (rosbag::MessageInstance const m : view) {
      auto pt = m.instantiate<geometry_msgs::Pose>();
      if (pt) {
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(pt->orientation, q);
        tf2::Matrix3x3 m(q);
        double unused1, unused2, yaw;
        m.getRPY(unused1, unused2, yaw);

        // Create docking station
        DockingStation ds;
        ds.id = generateNanoId();
        ds.name = "Docking Station";
        ds.active = true;
        ds.position = {pt->position.x, pt->position.y};
        ds.heading = yaw;
        target.docking_stations.push_back(ds);
      }
    }
  }

  bag.close();

  ROS_INFO_STREAM("Successfully converted legacy map to JSON with "
                  << target.areas.size() << " areas and " << target.docking_stations.size() << " docking stations");
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_map_service");
  ros::NodeHandle n;
  json_map_pub = n.advertise<std_msgs::String>("mower_map_service/json_map", 1, true);
  map_catalog_pub = n.advertise<std_msgs::String>("mower_map_service/map_catalog", 1, true);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mower_map_service/map", 10, true);
  map_server_viz_array_pub = n.advertise<visualization_msgs::MarkerArray>("mower_map_service/map_viz", 10, true);
  map_size_pub = n.advertise<xbot_msgs::MapSize>("mower_map_service/map_size", 10, true);

  rpc_provider.init();

  initializeCatalog(n);

  buildMap();

  ros::ServiceServer add_area_srv = n.advertiseService("mower_map_service/add_mowing_area", addMowingArea);
  ros::ServiceServer get_area_srv = n.advertiseService("mower_map_service/get_mowing_area", getMowingArea);
  ros::ServiceServer set_docking_point_srv = n.advertiseService("mower_map_service/set_docking_point", setDockingPoint);
  ros::ServiceServer get_docking_point_srv = n.advertiseService("mower_map_service/get_docking_point", getDockingPoint);
  ros::ServiceServer set_nav_point_srv = n.advertiseService("mower_map_service/set_nav_point", setNavPoint);
  ros::ServiceServer clear_nav_point_srv = n.advertiseService("mower_map_service/clear_nav_point", clearNavPoint);
  ros::ServiceServer clear_map_srv = n.advertiseService("mower_map_service/clear_map", clearMap);
  ros::ServiceServer apply_map_edit_srv = n.advertiseService("mower_map_service/apply_map_edit", applyMapEdit);
  ros::ServiceServer get_map_catalog_srv = n.advertiseService("mower_map_service/get_map_catalog", getMapCatalog);
  ros::ServiceServer create_map_srv = n.advertiseService("mower_map_service/create_map", createMap);
  ros::ServiceServer select_map_srv = n.advertiseService("mower_map_service/select_map", selectMap);
  ros::ServiceServer rename_map_srv = n.advertiseService("mower_map_service/rename_map", renameMap);
  ros::ServiceServer delete_map_srv = n.advertiseService("mower_map_service/delete_map", deleteMap);

  ros::spin();
  return 0;
}
