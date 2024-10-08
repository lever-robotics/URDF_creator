import React from "react";
import ParameterProps from "../ParameterProps";
import { IMU } from "../../../../Models/SensorsClass";
import ThreeScene from "../../../ThreeDisplay/ThreeScene";
import Frame from "../../../../Models/Frame";

function IMUParameters({ selectedObject, threeScene }: {threeScene: ThreeScene, selectedObject: Frame}) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        (selectedObject!.sensor as any)[name] = parseFloat(value);
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
