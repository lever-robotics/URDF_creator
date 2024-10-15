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
        threeScene.forceUpdateCode();
    };

    const sensorOptions = [
        { value: "", optionType: "Not a Sensor" },
        { value: "lidar", optionType: "Lidar" },
        { value: "camera", optionType: "Camera" },
        { value: "imu", optionType: "IMU" },
    ];

    return (
        <Section title="Sensor Parameters">
            <Property name="Sensor Type:">
                <Parameter
                    title=""
                    kind="select"
                    parameter="sensors"
                    value={selectedObject.sensorType}
                    handleChange={handleSensorTypeChange}
                    options={sensorOptions}
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
