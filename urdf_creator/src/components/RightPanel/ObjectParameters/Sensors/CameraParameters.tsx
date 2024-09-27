import React from "react";
import ParameterProps from "../ParameterProps";
import { Camera } from "../../../../Models/SensorsClass";


function CameraParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        threeScene.updateSensor(selectedObject, name, value);
    };

    const camera = selectedObject.sensor as Camera

    return (
        <div>
            <label>
                Camera Name:
                <input
                    type="text"
                    name="cameraName"
                    value={camera.cameraName}
                    onChange={handleChange}
                />
            </label>
            <label>
                Horizontal FOV:
                <input
                    type="number"
                    name="horizontal_fov"
                    value={camera.horizontal_fov}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Image Width:
                <input
                    type="number"
                    name="width"
                    value={camera.width}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Height:
                <input
                    type="number"
                    name="height"
                    value={camera.height}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Format:
                <input
                    type="text"
                    name="format"
                    value={camera.format}
                    onChange={handleChange}
                />
            </label>
            <label>
                Clip Near:
                <input
                    type="number"
                    name="near"
                    value={camera.near}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Clip Far:
                <input
                    type="number"
                    name="far"
                    value={camera.far}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={camera.gaussianNoise}
                    onChange={handleChange}
                />
            </label>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={camera.updateRate}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
            </label>
            <label>
                Image Topic Name:
                <input
                    type="text"
                    name="imageTopicName"
                    style={{ width: "200px" }}
                    value={camera.imageTopicName}
                    onChange={handleChange}
                />
            </label>
            <label>
                Camera Info Topic Name:
                <input
                    type="text"
                    name="cameraInfoTopicName"
                    style={{ width: "200px" }}
                    value={camera.cameraInfoTopicName}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default CameraParameters;
