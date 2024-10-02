import React from 'react';
import ParameterProps from '../ParameterProps';
import { Lidar } from '../../../../Models/SensorsClass';

function LidarParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        (selectedObject!.sensor as any)[name] = parseFloat(value);
    };

    const lidar = selectedObject.sensor as Lidar

    return (
        <div>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={lidar.updateRate}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
            </label>
            <label>
                Samples:
                <input
                    type="number"
                    name="samples"
                    value={lidar.samples}
                    onChange={handleChange}
                />
            </label>
            <label>
                Resolution:
                <input
                    type="number"
                    step="0.000001"
                    name="resolution"
                    value={lidar.resolution}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Min Angle:
                <input
                    type="number"
                    step="0.000001"
                    name="minAngle"
                    value={lidar.minAngle}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Max Angle:
                <input
                    type="number"
                    step="0.000001"
                    name="maxAngle"
                    value={lidar.maxAngle}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Min Range:
                <input
                    type="number"
                    step="0.000001"
                    name="minRange"
                    value={lidar.minRange}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Max Range:
                <input
                    type="number"
                    step="0.000001"
                    name="maxRange"
                    value={lidar.maxRange}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Range Resolution:
                <input
                    type="number"
                    step="0.000001"
                    name="rangeResolution"
                    value={lidar.rangeResolution}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Gaussian Noise Mean:
                <input
                    type="number"
                    step="0.000001"
                    name="mean"
                    value={lidar.mean}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise Stddev:
                <input
                    type="number"
                    step="0.000001"
                    name="stddev"
                    value={lidar.stddev}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default LidarParameters;
