import React, { useState, useEffect } from 'react';
import IMUParameters from '../Sensors/IMUParameters';
import CameraParameters from '../Sensors/CameraParameters';
import LidarParameters from '../Sensors/LidarParameters';
import { IMU, Camera, Lidar } from '../../../Models/SensorsClass';
// Import other sensor parameter components here

function SensorsParameters({ selectedObject, setSensor }) {
    const [localSensorType, setSensorType] = useState('');

    useEffect(() => {
        if (selectedObject) {
            if (selectedObject.sensor) {
                setSensorType(selectedObject.sensor.sensorType || '');
            } else {
                setSensorType('');
            }
        }
    }, [JSON.stringify(selectedObject.sensor)]);

    const handleSensorTypeChange = (e) => {
        switch (e.target.value) {
            case 'imu':
                const imu = new IMU();
                setSensor(selectedObject, imu);
                break;
            case 'camera':
                setSensor(selectedObject, new Camera());
                break;
            case 'lidar':
                setSensor(selectedObject, new Lidar());
                break;
            // Add cases for other sensor types here
            default:
                setSensor(selectedObject, null);
                setSensorType('');
                break;
        }
    };

    const renderSensorParameters = () => {
        switch (localSensorType) {
            case 'imu':
                return <IMUParameters selectedObject={selectedObject} sensorData={selectedObject.sensor} setSensor={setSensor} />;
            case 'camera':
                return <CameraParameters selectedObject={selectedObject} sensorData={selectedObject.sensor} setSensor={setSensor} />;
            case 'lidar':
                return <LidarParameters selectedObject={selectedObject} sensorData={selectedObject.sensor} setSensor={setSensor} />;
            // Add cases for other sensor types here
            default:
                return <div></div>;
        }
    };

    return (
        <div>
            <strong>Sensor Type:</strong>
            <select value={localSensorType} onChange={handleSensorTypeChange}>
                <option value="">Not a Sensor</option>
                <option value="lidar">Lidar</option>
                <option value="camera">Camera</option>
                <option value="imu">IMU</option>
                <option value="gps">GPS</option>
            </select>
            {renderSensorParameters()}
        </div>
    );
}


export default SensorsParameters;
