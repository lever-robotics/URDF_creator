import type React from "react";
import type Frame from "../../../../Models/Frame";
import { IMU } from "../../../../Models/SensorsClass";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import ParameterProps from "../ParameterProps";
import Parameter from "../Parameters/Parameter";
import Property from "../Parameters/Property";

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
        <>
            <Property>
                <Parameter
                    title="Update Rate:"
                    type="text"
                    name="updateRate"
                    value={imu.updateRate}
                    units="Hz"
                    onChange={handleChange}
                />
            </Property>
            <Property>
                <Parameter
                    title="Mean:"
                    type="text"
                    name="mean"
                    value={imu.mean}
                    onChange={handleChange}
                />
            </Property>
            <Property>
                <Parameter
                    title="Standard Deviation:"
                    type="text"
                    name="stddev"
                    value={imu.stddev}
                    onChange={handleChange}
                />
            </Property>
            <Property>
                <Parameter
                    title="Gaussian Noise:"
                    type="text"
                    name="gaussianNoise"
                    value={imu.gaussianNoise}
                    onChange={handleChange}
                />
            </Property>
        </>
    );
}

export default IMUParameters;
