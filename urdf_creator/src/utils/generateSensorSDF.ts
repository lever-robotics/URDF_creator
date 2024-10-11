import type Frame from "../Models/Frame";
import { Camera, IMU, Lidar } from "../Models/SensorsClass";

export const generateSensorSDF = (selectedObject: Frame) => {
    const sensorType = selectedObject.sensor.type;
    let sensorXML = "";
    switch (sensorType) {
        case "imu":
            sensorXML = generateIMUPluginXML(selectedObject);
            break;
        case "camera":
            sensorXML = generateCameraPluginXML(selectedObject);
            break;
        case "lidar":
            sensorXML = generateLidarPluginXML(selectedObject);
            break;
        // Add cases for other sensor types heres
        default:
            console.error(`Unknown sensor type: ${sensorType}`);
            break;
    }

    return sensorXML;
};

const generateIMUPluginXML = (selectedObject: Frame) => {
    const { sensor, name } = selectedObject;
    if (sensor instanceof IMU) {
        const { type, alwaysOn, updateRate, mean, stddev } = sensor;
        return `
        <sensor name="${type}" type="imu">
            <always_on>${alwaysOn}</always_on>
            <update_rate>${updateRate}</update_rate>
            <imu>
                <angular_velocity>
                    <x>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </z>
                </angular_velocity>
                <linear_acceleration>
                    <x>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="gaussian">
                            <mean>${mean}</mean>
                            <stddev>${stddev}</stddev>
                        </noise>
                    </z>
                </linear_acceleration>
            </imu>
            <plugin name="imu_plugin_for_${name.replace("-", "_")}" filename="libgazebo_ros_imu_sensor.so">
                <ros>
                    <remapping>~/out:=${name.replace("-", "_")}/imu</remapping>
                </ros>
            </plugin>
        </sensor>
    `;
    }
    console.error("Sensor is not an instance of IMU");
    return "";
};

const generateCameraPluginXML = (selectedObject: Frame) => {
    const { sensor, name } = selectedObject;

    if (sensor instanceof Camera) {
        const {
            cameraName,
            alwaysOn,
            updateRate,
            horizontal_fov,
            width,
            height,
            format,
            near,
            far,
            mean,
            stddev,
        } = sensor;

        return `
            <sensor name="${name.replace("-", "_")}_${cameraName}" type="camera">
                <always_on>${alwaysOn}</always_on>
                <visualize>true</visualize>
                <update_rate>${updateRate}</update_rate>
                <camera name="${name.replace("-", "_")}_camera">
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
                        <mean>${mean}</mean>
                        <stddev>${stddev}</stddev>
                    </noise>
                </camera>
                <plugin name="camera_driver_for_${name.replace("-", "_")}" filename="libgazebo_ros_camera.so">
                    <ros>
                        <!-- <namespace>test_cam</namespace> -->
                        <!-- <remapping>image_raw:=image_demo</remapping> -->
                        <!-- <remapping>camera_info:=camera_info_demo</remapping> -->
                    </ros>
                    <!-- <camera_name>omit so it defaults to sensor name</camera_name> -->
                    <!-- <frame_name>omit so it defaults to link name</frameName> -->
                    <!-- <hack_baseline>0.07</hack_baseline> -->
                </plugin>
            </sensor>
        `;
    }
    console.error("Sensor is not an instance of Camera");
    return "";
};

const generateLidarPluginXML = (selectedObject: Frame) => {
    const { sensor, name } = selectedObject;
    if (sensor instanceof Lidar) {
        const {
            type,
            alwaysOn,
            updateRate,
            pose,
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

        return `
        <sensor name="${name.replace("-", "_")}_${type}" type="ray">
            <always_on>${alwaysOn}</always_on>
            <visualize>true</visualize>
            <pose>${pose}</pose>
            <update_rate>${updateRate}</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>${samples}</samples>
                        <resolution>${resolution}</resolution>
                        <min_angle>${minAngle}</min_angle>
                        <max_angle>${maxAngle}</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>${minRange}</min>
                    <max>${maxRange}</max>
                    <resolution>${rangeResolution}</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>${mean}</mean>
                    <stddev>${stddev}</stddev>
                </noise>
            </ray>
            <plugin name="laserscan_for_${name.replace("-", "_")}" filename="libgazebo_ros_ray_sensor.so">
                <ros>
                    <remapping>~/out:=${name.replace("-", "_")}/scan</remapping>
                </ros>
                <output_type>sensor_msgs/LaserScan</output_type>
                <frame_name>${name}</frame_name>
            </plugin>
        </sensor>
    `;
    }

    console.error("Sensor is not an instance of Lidar");
    return "";
};
