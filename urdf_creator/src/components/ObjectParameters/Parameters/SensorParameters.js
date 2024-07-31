import ToggleSection from "../ToggleSection";
import IMUParameters from "../Sensors/IMUParameters";
import CameraParameters from "../Sensors/CameraParameters";
import LidarParameters from "../Sensors/LidarParameters";

// Import other sensor parameter components here

function SensorsParameters({ selectedObject, stateFunctions }) {
    
    const handleSensorTypeChange = (e) => {
        const type = e.target.value;
        stateFunctions.setSensor(selectedObject, type);
    };

    return (
        <ToggleSection title="Sensor Parameters">
            <strong>Sensor Type:</strong>
            <select value={selectedObject.sensorType} onChange={handleSensorTypeChange}>
                <option value="">Not a Sensor</option>
                <option value="lidar">Lidar</option>
                <option value="camera">Camera</option>
                <option value="imu">IMU</option>
                <option value="gps">GPS</option>
            </select>
            <SensorParams
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
        </ToggleSection>
    );
}

function SensorParams({ selectedObject, stateFunctions }) {
    switch (selectedObject.sensorType) {
        case "imu":
            return (
                <IMUParameters
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            );
        case "camera":
            return (
                <CameraParameters
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            );
        case "lidar":
            return (
                <LidarParameters
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            );
        // Add cases for other sensor types here
        default:
            return <div></div>;
    }
}

export default SensorsParameters;
