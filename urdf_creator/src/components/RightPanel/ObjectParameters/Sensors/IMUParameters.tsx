import React from "react";
import ParameterProps from "../ParameterProps";
import { IMU } from "../../../../Models/SensorsClass";

function IMUParameters({ selectedObject, stateFunctions }: ParameterProps) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        stateFunctions.updateSensor(selectedObject, name, value);
    };

    const imu = selectedObject.sensor as IMU

    return (
        <div>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={imu.updateRate}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
            </label>
            <label>
                Mean:
                <input
                    type="number"
                    name="mean"
                    value={imu.mean}
                    onChange={handleChange}
                />
            </label>
            <label>
                Standard Deviation:
                <input
                    type="number"
                    name="stddev"
                    value={imu.stddev}
                    onChange={handleChange}
                />
            </label>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={imu.gaussianNoise}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default IMUParameters;
