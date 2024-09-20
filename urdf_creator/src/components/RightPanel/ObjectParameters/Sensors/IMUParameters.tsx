import React from "react";
import ParameterProps from "../ParameterProps";

function IMUParameters({ selectedObject, stateFunctions }: ParameterProps) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
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
                Mean:
                <input
                    type="number"
                    name="mean"
                    value={selectedObject.sensor.mean}
                    onChange={handleChange}
                />
            </label>
            <label>
                Standard Deviation:
                <input
                    type="number"
                    name="stddev"
                    value={selectedObject.sensor.stddev}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={selectedObject.sensor.gaussianNoise}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default IMUParameters;
