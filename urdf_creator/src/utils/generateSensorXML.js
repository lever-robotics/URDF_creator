// generateSensorXML.js
import * as THREE from 'three';

export const generateSensorXML = (selectedObject) => {

    const sensorType = selectedObject.userData.sensorType;
    let sensorXML = '';
    switch (sensorType) {
        case 'imu':
            sensorXML = generateIMUXML(selectedObject);
            break;
        case 'camera':
            sensorXML = generateCameraXML(selectedObject);
            break;
        // Add cases for other sensor types here
        default:
            console.error(`Unknown sensor type: ${sensorType}`);
            break;
    }

    return sensorXML;
};

const generateIMUXML = (selectedObject) => {
    // Generate IMU-specific XML here
    return `
        <sensor type="imu" name="${selectedObject.userData.name}">
            <always_on>${selectedObject.userData.alwaysOn}</always_on>
            <update_rate>${selectedObject.userData.updateRate}</update_rate>
            <imu>
                <noise>
                    <type>gaussian</type>
                    <mean>${selectedObject.userData.mean}</mean>
                    <stddev>${selectedObject.userData.stddev}</stddev>
                </noise>
            </imu>
        </sensor>
    `;
};

const generateCameraXML = (selectedObject) => {
    // Generate Camera-specific XML here
    return `
        <sensor type="camera" name="${selectedObject.userData.name}">
            <camera>
                <horizontal_fov>${selectedObject.userData.horizontal_fov}</horizontal_fov>
                <image>
                    <width>${selectedObject.userData.width}</width>
                    <height>${selectedObject.userData.height}</height>
                    <format>${selectedObject.userData.format}</format>
                </image>
                <clip>
                    <near>${selectedObject.userData.near}</near>
                    <far>${selectedObject.userData.far}</far>
                </clip>
            </camera>
        </sensor>
    `;
};

// Add similar functions for other sensors as needed
