# Application Functionality Overview

Two forms will be created: a **Public URDF Form** and a **Sensor Form**. These forms will be prefilled with existing data when a user is editing an entry or left blank when creating a new one.

## RoboEverything.com

### 1. **Display All Public URDF Models**
   - **Purpose**: Retrieve and display a list of all public URDF models.
   - **Functionality**:
     - On page load, fetch and display URDF model IDs, names, images, and brief descriptions.
     - Each model entry is clickable, opening a detailed view with all related information.

### 2. **Display Detailed URDF Model Information**
   - **Purpose**: Show detailed information for a selected URDF model.
   - **Functionality**:
     - Upon clicking a model, open a detailed view showing all fields (e.g., name, description, GLTF, license, creator).
     - Includes an **Edit** button, which opens the **Public URDF Form** prefilled with data points for the selected model.

### 3. **URDF Model Editor (Add/Edit)**
   - **Purpose**: Allow users to edit an existing URDF or add a new one.
   - **Functionality**:
     - **Edit Mode**: Prefilled with the selected URDF’s data in the **Public URDF Form**.
     - **Add Mode**: Opens the form blank for new URDF creation.
     - Upon submission, the request is marked as **“Processing for Review”** and added to a review queue.

### 4. **Display and Edit Sensors**
   - **Purpose**: Similar to URDFs, display and edit sensor details.
   - **Functionality**:
     - Pull and display all sensors.
     - Each entry shows full information upon click, with an **Edit** button for opening the **Sensor Form** prefilled with data or blank for new entries.
     - Submission goes through a review process as with URDFs.

**Note**: Other edits to the database will currently be managed manually by administrators.

## URDF Creator

### 1. **Display List of Public URDFs**
   - **Purpose**: Fetch and display a list of public URDFs.
   - **Functionality**:
     - Upon load, the application shows all URDFs available for download and use.
     - Clicking a URDF downloads it and integrates it into the scene within the application.

### 2. **Display List of Sensors**
   - **Purpose**: Same functionality as URDFs, but for sensors.
   - **Functionality**:
     - Displays a list of sensors with the ability to download and place each sensor within the scene.

### 3. **Export to Public Repository**
   - **Purpose**: Facilitate adding URDFs to the public repository on RoboEverything.com.
   - **Functionality**:
     - Includes an **Export** button that, when clicked, sends URDF data (including the GLTF file) to RoboEverything.com’s **Public URDF Form**.
     - Prefills as much information as possible, including the GLTF file, and prompts the user to fill in additional details such as description, license, and long name.
