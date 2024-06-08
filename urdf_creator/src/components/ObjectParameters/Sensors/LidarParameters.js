import React from 'react';
import { Lidar } from '../../../Models/SensorsClass';

function LidarParameters({ selectedObject, sensorData, setSensor }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        setSensor(selectedObject, new Lidar({ ...sensorData, [name]: value }));
    };

    return (
        <div>
            <label>
                Sensor Type:
                <input
                    type="text"
                    name="sensorType"
                    value={sensorData.sensorType || 'ray'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Always On:
                <input
                    type="checkbox"
                    name="alwaysOn"
                    checked={sensorData.alwaysOn !== undefined ? sensorData.alwaysOn : true}
                    onChange={(e) => handleChange({ target: { name: 'alwaysOn', value: e.target.checked } })}
                />
            </label>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={sensorData.updateRate || 5}
                    onChange={handleChange}
                />
            </label>
            <label>
                Pose:
                <input
                    type="text"
                    name="pose"
                    value={sensorData.pose || '-0.064 0 0.121 0 0 0'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Samples:
                <input
                    type="number"
                    name="samples"
                    value={sensorData.samples || 360}
                    onChange={handleChange}
                />
            </label>
            <label>
                Resolution:
                <input
                    type="number"
                    step="0.000001"
                    name="resolution"
                    value={sensorData.resolution || 1.000000}
                    onChange={handleChange}
                />
            </label>
            <label>
                Min Angle:
                <input
                    type="number"
                    step="0.000001"
                    name="minAngle"
                    value={sensorData.minAngle || 0.000000}
                    onChange={handleChange}
                />
            </label>
            <label>
                Max Angle:
                <input
                    type="number"
                    step="0.000001"
                    name="maxAngle"
                    value={sensorData.maxAngle || 6.280000}
                    onChange={handleChange}
                />
            </label>
            <label>
                Min Range:
                <input
                    type="number"
                    step="0.000001"
                    name="minRange"
                    value={sensorData.minRange || 0.120000}
                    onChange={handleChange}
                />
            </label>
            <label>
                Max Range:
                <input
                    type="number"
                    step="0.000001"
                    name="maxRange"
                    value={sensorData.maxRange || 3.5}
                    onChange={handleChange}
                />
            </label>
            <label>
                Range Resolution:
                <input
                    type="number"
                    step="0.000001"
                    name="rangeResolution"
                    value={sensorData.rangeResolution || 0.015000}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise Mean:
                <input
                    type="number"
                    step="0.000001"
                    name="mean"
                    value={sensorData.mean || 0.0}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise Stddev:
                <input
                    type="number"
                    step="0.000001"
                    name="stddev"
                    value={sensorData.stddev || 0.01}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default LidarParameters;
