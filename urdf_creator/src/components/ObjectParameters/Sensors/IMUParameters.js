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
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={sensorData.updateRate || 100}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
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
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={sensorData.gaussianNoise || 0.01}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default IMUParameters;
