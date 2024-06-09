// generateSensorXML.js
export const generateSensorXML = (selectedObject) => {

    const sensorType = selectedObject.userData.sensor.sensorType;
    let sensorXML = '';
    switch (sensorType) {
        case 'imu':
            sensorXML = generateIMUXML(selectedObject);
            break;
        case 'camera':
            sensorXML = generateCameraXML(selectedObject);
            break;
        case 'lidar':
            sensorXML = generateLidarXML(selectedObject);
            break;
        // Add cases for other sensor types here
        default:
            console.error(`Unknown sensor type: ${sensorType}`);
            break;
    }

    return sensorXML;
};

const generateIMUXML = (selectedObject) => {
    const { sensor } = selectedObject.userData;
    const { sensorType, xyzOffsets, rpyOffsets, alwaysOn, updateRate, mean, stddev } = sensor;

    return `
    <sensor type="${sensorType}" name="${sensor.sensorType}">
        <always_on>${alwaysOn}</always_on>
        <update_rate>${updateRate}</update_rate>
        <pose>${xyzOffsets + selectedObject.position.z + 0.1} ${rpyOffsets}</pose>
        <imu>
            <noise>
                <type>gaussian</type>
                <mean>${mean}</mean>
                <stddev>${stddev}</stddev>
            </noise>
        </imu>
    </sensor>
`;
};


const generateCameraXML = (selectedObject) => {
    const { sensor } = selectedObject.userData;
    const { sensorType, gaussianNoise, xyzOffsets, rpyOffsets, alwaysOn, updateRate, cameraName, imageTopicName, cameraInfoTopicName, horizontal_fov, width, height, format, near, far } = sensor;

    return `
    <sensor type="${sensorType}" name="${cameraName}">
        <always_on>${alwaysOn}</always_on>
        <update_rate>${updateRate}</update_rate>
        <pose>${xyzOffsets} ${rpyOffsets}</pose>
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
};

const generateLidarXML = (selectedObject) => {
    const { sensor } = selectedObject.userData;
    const { sensorType, alwaysOn, updateRate, pose, samples, resolution, minAngle, maxAngle, minRange, maxRange, rangeResolution, mean, stddev } = sensor;

    return `
    <sensor type="${sensorType}" name="${sensor.sensorType}">
        <always_on>${alwaysOn}</always_on>
        <update_rate>${updateRate}</update_rate>
        <pose>${pose}</pose>
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
};