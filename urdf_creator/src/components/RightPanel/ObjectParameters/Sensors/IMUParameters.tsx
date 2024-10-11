import type React from "react";
import type Frame from "../../../../Models/Frame";
import { IMU } from "../../../../Models/SensorsClass";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import ParameterProps from "../ParameterProps";

function IMUParameters({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const imu = selectedObject.sensor;
    if (!(imu instanceof IMU)) return;

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        const newValue = Number.parseFloat(value);
        switch (name) {
            case "updateRate":
                imu.updateRate = newValue;
                break;
            case "gaussianNoise":
                imu.gaussianNoise = newValue;
                break;
            case "mean":
                imu.mean = newValue;
                break;
            case "stddev":
                imu.stddev = newValue;
                break;
        }
    };

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
