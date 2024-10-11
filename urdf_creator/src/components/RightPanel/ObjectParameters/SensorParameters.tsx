import type React from "react";
import type Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import styles from "./ObjectParameters.module.css";
import ParameterProps from "./ParameterProps";
import Parameter from "./Parameters/Parameter";
import Property from "./Parameters/Property";
import Section from "./Parameters/Section";
import CameraParameters from "./Sensors/CameraParameters";
import IMUParameters from "./Sensors/IMUParameters";
import LidarParameters from "./Sensors/LidarParameters";

function SensorsParameters({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;

    const handleSensorTypeChange = (
        e: React.ChangeEvent<HTMLSelectElement>,
    ) => {
        const type = e.target.value;
        selectedObject.sensorType = type;
        threeScene.forceUpdateBoth();
    };

    // <Parameter
    //                         title=""
    //                         type="select"
    //                         value={selectedObject.jointType}
    //                         onSelectChange={handleJointTypeChange}
    //                         className={styles.select}
    //                         options={[
    //                             "Fixed",
    //                             "Revolute",
    //                             "Continuous",
    //                             "Prismatic",
    //                         ]}>
    //                     </Parameter>

    return (
        <Section title="Sensor Parameters">
            <Property name="Sensor Type:">
                <Parameter
                    title=""
                    type="select"
                    value={selectedObject.sensorType}
                    onSelectChange={handleSensorTypeChange}
                    options={[
                        { value: "", option: "Not a Sensor" },
                        { value: "lidar", option: "Lidar" },
                        { value: "camera", option: "Camera" },
                        { value: "imu", option: "IMU" },
                    ]}
                />
            </Property>
            <SensorParams
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
        </Section>
    );
}

function SensorParams({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    switch (selectedObject.sensorType) {
        case "imu":
            return (
                <IMUParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
            );
        case "camera":
            return (
                <CameraParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
            );
        case "lidar":
            return (
                <LidarParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
            );
        // Add cases for other sensor types here
        default:
            return <div />;
    }
}

export default SensorsParameters;
