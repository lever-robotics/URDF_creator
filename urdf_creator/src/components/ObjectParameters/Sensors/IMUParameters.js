import React from 'react';
import { IMU } from '../../../Models/SensorsClass';

function IMUParameters({ selectedObject, sensorData, setSensor }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        setSensor(selectedObject, new IMU({ ...sensorData, [name]: value }));
    };

    return (
        <div>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={sensorData.gaussianNoise || 0.01}
                    onChange={handleChange}
                />
            </label>
            <label>
                XYZ Offsets:
                <input
                    type="text"
                    name="xyzOffsets"
                    value={sensorData.xyzOffsets || '0 0 0'}
                    onChange={handleChange}
                />
            </label>
            <label>
                RPY Offsets:
                <input
                    type="text"
                    name="rpyOffsets"
                    value={sensorData.rpyOffsets || '0 0 0'}
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
                    value={sensorData.updateRate || 100}
                    onChange={handleChange}
                />
            </label>
            <label>
                Mean:
                <input
                    type="number"
                    name="mean"
                    value={sensorData.mean || 0}
                    onChange={handleChange}
                />
            </label>
            <label>
                Standard Deviation:
                <input
                    type="number"
                    name="stddev"
                    value={sensorData.stddev || 0}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default IMUParameters;
