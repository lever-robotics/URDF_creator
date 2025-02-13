import type Frame from "../Models/Frame";
import { Camera, IMU, Lidar } from "../Models/SensorsClass";
import { quaternionToRPY } from "./quaternionToRPY";

// generateSensorXML.js
export const generateSensorXML = (selectedObject: Frame) => {
    const sensorType = selectedObject.sensor.type;
    let sensorXML = "";
    switch (sensorType) {
        case "imu":
            sensorXML = generateIMUXML(selectedObject);
            break;
        case "camera":
            sensorXML = generateCameraXML(selectedObject);
            break;
        case "lidar":
            sensorXML = generateLidarXML(selectedObject);
            break;
        // Add cases for other sensor types here
        default:
            // console.error(`Unknown sensor type: ${sensorType}`);
            break;
    }

    return sensorXML;
};

const generateIMUXML = (selectedObject: Frame) => {
    const { sensor } = selectedObject;
    if (sensor instanceof IMU) {
        const {
            type,
            xyzOffsets,
            rpyOffsets,
            alwaysOn,
            updateRate,
            mean,
            stddev,
        } = sensor;

        const { x, y, z } = selectedObject.position;
        const xyzOffsets_mesh = `${x} ${y} ${z}`;
        const rpyOffsets_mesh = quaternionToRPY(selectedObject.quaternion);

        //currently settinng the origin of the sensor to that of the mesh

        return `
        <sensor type="${type}" name="${type}">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
            <origin>${xyzOffsets_mesh} ${rpyOffsets_mesh}</origin>
            <imu>
                <noise>
                    <type>gaussian</type>
                    <mean>${mean}</mean>
                    <stddev>${stddev}</stddev>
                </noise>
            </imu>
        </sensor>
    `;
    }

    return "";
};

const generateCameraXML = (selectedObject: Frame) => {
    const { sensor } = selectedObject;
    if (sensor instanceof Camera) {
        const {
            type,
            gaussianNoise,
            xyzOffsets,
            rpyOffsets,
            alwaysOn,
            updateRate,
            cameraName,
            imageTopicName,
            cameraInfoTopicName,
            horizontal_fov,
            width,
            height,
            format,
            near,
            far,
        } = sensor;

        //currently settinng the origin of the sensor to that of the mesh
        const { x, y, z } = selectedObject.position;
        const xyzOffsets_mesh = `${x} ${y} ${z}`;
        const rpyOffsets_mesh = quaternionToRPY(selectedObject.quaternion);

        return `
        <sensor name="${cameraName}" type="${type}">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
            <origin>${xyzOffsets_mesh} ${rpyOffsets_mesh}</origin>
            <camera>
                <horizontal_fov>${horizontal_fov}</horizontal_fov>
                <image>
                    <width>${width}</width>
                    <height>${height}</height>
                    <format>${format}</format>
                </image>
                <clip>
                    <near>${near}</near>
                    <far>${far}</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>${gaussianNoise}</mean>
                </noise>
            </camera>
        </sensor>
    `;
    }

    return "";
};

const generateLidarXML = (selectedObject: Frame) => {
    const { sensor } = selectedObject;
    if (sensor instanceof Lidar) {
        const {
            type,
            alwaysOn,
            updateRate,
            samples,
            resolution,
            minAngle,
            maxAngle,
            minRange,
            maxRange,
            rangeResolution,
            mean,
            stddev,
        } = sensor;

        //currently settinng the origin of the sensor to that of the mesh

        const { x, y, z } = selectedObject.position;
        const xyzOffsets_mesh = `${x} ${y} ${z}`;
        const rpyOffsets_mesh = quaternionToRPY(selectedObject.quaternion);

        return `
        <sensor type="${type}" name="${sensor.type}">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
            <origin>${xyzOffsets_mesh} ${rpyOffsets_mesh}</origin>
            <ray>
                <scan>
                    <horizontal>
                        <samples>${samples}</samples>
                        <resolution>${resolution}</resolution>
                        <min_angle>${minAngle}</min_angle>
                        <max_angle>${maxAngle}</max_angle>
                    </horizontal>
                    <range>
                        <min>${minRange}</min>
                        <max>${maxRange}</max>
                        <resolution>${rangeResolution}</resolution>
                    </range>
                </scan>
                <noise>
                    <type>gaussian</type>
                    <mean>${mean}</mean>
                    <stddev>${stddev}</stddev>
                </noise>
            </ray>
        </sensor>
    `;
    }

    return "";
};
