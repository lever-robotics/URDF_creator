export const generateSensorSDF = (selectedObject) => {

    const sensorType = selectedObject.userData.sensorType;
    let sensorXML = '';
    switch (sensorType) {
        case 'imu':
            sensorXML = generateIMUPluginXML(selectedObject);
            break;
        case 'camera':
            sensorXML = generateCameraPluginXML(selectedObject);
            break;
        // Add cases for other sensor types here
        default:
            console.error(`Unknown sensor type: ${sensorType}`);
            break;
    }

    return sensorXML;
};

const generateIMUPluginXML = (selectedObject) => {
    const { sensor } = selectedObject.userData;
    const { sensorType, gaussianNoise, xyzOffsets, rpyOffsets, alwaysOn, updateRate, mean, stddev } = sensor;

    return `
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
            <imu>
                <noise>
                    <type>gaussian</type>
                    <mean>${mean}</mean>
                    <stddev>${stddev}</stddev>
                </noise>
            </imu>
            <ros>
                <namespace>${sensorType}</namespace>
                <remapping>~/out:=/imu/data</remapping>
            </ros>
            <pose>${xyzOffsets} ${rpyOffsets}</pose>
        </plugin>
    `;
};

const generateCameraPluginXML = (selectedObject) => {
    const { sensor } = selectedObject.userData;
    const { sensorType, gaussianNoise, xyzOffsets, rpyOffsets, alwaysOn, updateRate, cameraName, imageTopicName, cameraInfoTopicName, horizontal_fov, width, height, format, near, far } = sensor;

    return `
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
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
            <ros>
                <namespace>${sensorType}</namespace>
                <remapping>~/image_raw:=${imageTopicName}</remapping>
                <remapping>~/camera_info:=${cameraInfoTopicName}</remapping>
            </ros>
            <pose>${xyzOffsets} ${rpyOffsets}</pose>
        </plugin>
    `;
};
