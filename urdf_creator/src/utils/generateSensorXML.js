// generateSensorXML.js
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
    const { name, alwaysOn, updateRate, mean, stddev } = selectedObject.userData;

    return `
        <sensor type="imu" name="${name}">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
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
    const { name, horizontal_fov, width, height, format, near, far } = selectedObject.userData;

    return `
        <sensor type="camera" name="${name}">
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
            </camera>
        </sensor>
    `;
};
