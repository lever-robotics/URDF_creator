import Section from "../Section";
import IMUParameters from "../Sensors/IMUParameters";
import CameraParameters from "../Sensors/CameraParameters";
import LidarParameters from "../Sensors/LidarParameters";
import React from "react";
import ParameterProps from "../ParameterProps";

function SensorsParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    
    const handleSensorTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        const type = e.target.value;
        selectedObject.sensorType = type;
        threeScene.forceUpdateBoth();
    };

    return (
        <Section title="Sensor Parameters">
            <strong>Sensor Type:</strong>
            <select value={selectedObject.sensorType} onChange={handleSensorTypeChange}>
                <option value="">Not a Sensor</option>
                <option value="lidar">Lidar</option>
                <option value="camera">Camera</option>
                <option value="imu">IMU</option>
                {/* <option value="gps">GPS</option> */}
            </select>
            <SensorParams
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
        </Section>
    );
}

function SensorParams({ selectedObject, threeScene }: ParameterProps) {
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
            return <div></div>;
    }
}

export default SensorsParameters;
