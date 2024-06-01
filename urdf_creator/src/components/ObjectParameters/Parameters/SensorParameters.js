import React, { useState, useEffect } from 'react';
import IMUParameters from '../Sensors/IMUParameters';
import CameraParameters from '../Sensors/CameraParameters';
// Import other sensor parameter components here

function SensorsParameters({ selectedObject, onUpdate }) {
    const [isSensor, setIsSensor] = useState(false);
    const [sensorType, setSensorType] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setIsSensor(selectedObject.userData?.isSensor);
            setSensorType(selectedObject.userData.sensorType);
        }
    }, [JSON.stringify(selectedObject.userData)]);

    const handleSensorChange = () => {
        setIsSensor(!isSensor);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.isSensor = !isSensor;
        onUpdate(updatedObject);
    };

    const handleSensorTypeChange = (e) => {
        setSensorType(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.sensorType = e.target.value;
        onUpdate(updatedObject);
    };

    const handleSensorParamsChange = (newUserData) => {
        const updatedObject = { ...selectedObject, userData: newUserData };
        onUpdate(updatedObject);
    };

    const renderSensorParameters = () => {
        switch (sensorType) {
            case 'imu':
                return <IMUParameters userData={selectedObject.userData} onChange={handleSensorParamsChange} />;
            case 'camera':
                return <CameraParameters userData={selectedObject.userData} onChange={handleSensorParamsChange} />;
            // Add cases for other sensor types here
            default:
                return null;
        }
    };

    return (
        <div>
            <label>
                <input
                    type="checkbox"
                    checked={isSensor}
                    onChange={handleSensorChange}
                />
                Is Sensor
            </label>
            {isSensor && (
                <div>
                    <strong>Sensor Type:</strong>
                    <select value={sensorType} onChange={handleSensorTypeChange}>
                        <option value="">Select a sensor</option>
                        <option value="lidar">Lidar</option>
                        <option value="camera">Camera</option>
                        <option value="imu">IMU</option>
                        <option value="gps">GPS</option>
                    </select>
                    {renderSensorParameters()}
                </div>
            )}
        </div>
    );
}

export default SensorsParameters;
