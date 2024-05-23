import React, { useState, useEffect } from 'react';

function SensorsParameters({ selectedObject, onUpdate }) {
    const [isSensor, setIsSensor] = useState(false);
    const [sensorType, setSensorType] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setIsSensor(selectedObject.userData?.isSensor || false);
            setSensorType(selectedObject.userData?.sensorType || '');
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
                </div>
            )}
        </div>
    );
}

export default SensorsParameters;
