import React from 'react';

function LidarParameters({ selectedObject, stateFunctions }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        stateFunctions.updateSensor(selectedObject, name, value);
    };

    return (
        <div>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={selectedObject.sensor.updateRate}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
            </label>
            <label>
                Samples:
                <input
                    type="number"
                    name="samples"
                    value={selectedObject.sensor.samples}
                    onChange={handleChange}
                />
            </label>
            <label>
                Resolution:
                <input
                    type="number"
                    step="0.000001"
                    name="resolution"
                    value={selectedObject.sensor.resolution}
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
                    value={selectedObject.sensor.minAngle}
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
                    value={selectedObject.sensor.maxAngle}
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
                    value={selectedObject.sensor.minRange}
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
                    value={selectedObject.sensor.maxRange}
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
                    value={selectedObject.sensor.rangeResolution}
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
                    value={selectedObject.sensor.mean}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise Stddev:
                <input
                    type="number"
                    step="0.000001"
                    name="stddev"
                    value={selectedObject.sensor.stddev}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default LidarParameters;
