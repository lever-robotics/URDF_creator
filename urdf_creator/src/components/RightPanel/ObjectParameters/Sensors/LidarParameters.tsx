import type React from "react";
import type Frame from "../../../../Models/Frame";
import { Lidar } from "../../../../Models/SensorsClass";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import ParameterProps from "../ParameterProps";

function LidarParameters({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const lidar = selectedObject.sensor;
    if (!(lidar instanceof Lidar)) return;

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        const newValue = Number.parseFloat(value);
        switch (name) {
            case "updateRate":
                lidar.updateRate = newValue;
                break;
            case "samples":
                lidar.samples = newValue;
                break;
            case "resolution":
                lidar.resolution = newValue;
                break;
            case "minAngle":
                lidar.minAngle = newValue;
                break;
            case "maxAngle":
                lidar.maxAngle = newValue;
                break;
            case "minRange":
                lidar.minRange = newValue;
                break;
            case "maxRange":
                lidar.maxRange = newValue;
                break;
            case "rangeResolution":
                lidar.rangeResolution = newValue;
                break;
            case "mean":
                lidar.mean = newValue;
                break;
            case "stddev":
                lidar.stddev = newValue;
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
