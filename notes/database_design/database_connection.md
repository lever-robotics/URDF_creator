# Database Interface Functions

The following functions provide access to retrieve, download, edit, and create entries for URDFs and Sensors in the database. **Note**: Each function requires token verification to ensure authorized access before processing requests.

## 1. `getPublicURDFs`

- **Purpose**: Retrieves a list of all public URDFs for display purposes.
- **Inputs**: None.
- **Outputs**:
  - A list of public URDFs, each containing:
    - `urdf_id`: Unique identifier for the URDF.
    - `name_long`: Full name of the URDF.
    - `image`: URL of the URDF's image.
    - `description`: Brief description of the URDF.
    - `weblinks`: Array of associated URLs for additional resources.

## 2. `downloadURDF`

- **Purpose**: Downloads the GLTF file and all associated STL/DAE files for a specific URDF, and increments its download count.
- **Inputs**:
  - `urdf_id`: Unique identifier of the URDF to be downloaded.
- **Outputs**:
  - `gltf_file`: The GLTF file for the URDF.
  - `mesh_files`: Array of connected STL or DAE files.
  - An updated `download_count` for the URDF.

## 3. `getSensorsList`

- **Purpose**: Retrieves a list of all sensors for display purposes.
- **Inputs**: None.
- **Outputs**:
  - A list of sensors, each containing:
    - `sensor_id`: Unique identifier for the sensor.
    - `name_long`: Full name of the sensor.
    - `image`: URL of the sensor's image.
    - `description`: Brief description of the sensor.
    - `purchase_links`: Array of URLs for purchasing the sensor.

## 4. `downloadSensor`

- **Purpose**: Downloads the GLTF file for a specific sensor.
- **Inputs**:
  - `sensor_id`: Unique identifier of the sensor to be downloaded.
- **Outputs**:
  - `gltf_file`: The GLTF file for the sensor.

## 5. `createPublicURDF`

- **Purpose**: Creates a new public URDF entry in the database.
- **Inputs**:
  - All necessary URDF data, including:
    - `name_long`, `name_short`, `urdf_file`, `sdf_file`, `gltf_file`, `image`, `description`, `license`, `creator_id`, `date_created`, `gltf_version`, `weblinks`, `connected_files`.
- **Outputs**:
  - Confirmation of successful creation and the new `urdf_id`.

## 6. `editPublicURDF`

- **Purpose**: Edits an existing public URDF entry.
- **Inputs**:
  - `urdf_id`: Unique identifier of the URDF to be edited.
  - Updated data fields (as needed).
- **Outputs**:
  - Confirmation of successful update.

## 7. `createSensor`

- **Purpose**: Creates a new sensor entry in the database.
- **Inputs**:
  - All necessary sensor data, including:
    - `name_long`, `name_short`, `sensor_type`, `image`, `description`, `date_manufactured`, `date_created`, `gltf_file`, `purchase_links`.
- **Outputs**:
  - Confirmation of successful creation and the new `sensor_id`.

## 8. `editSensor`

- **Purpose**: Edits an existing sensor entry.
- **Inputs**:
  - `sensor_id`: Unique identifier of the sensor to be edited.
  - Updated data fields (as needed).
- **Outputs**:
  - Confirmation of successful update.

**Note**: Ensure each function verifies tokens to confirm user authorization before proceeding with any database operations.
