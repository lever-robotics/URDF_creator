import React from "react";
import ParameterProps from "../ParameterProps";


function CameraParameters({ selectedObject, stateFunctions }: ParameterProps) {
    if (!selectedObject) return;
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        stateFunctions.updateSensor(selectedObject, name, value);
    };

    return (
        <div>
            <label>
                Camera Name:
                <input
                    type="text"
                    name="cameraName"
                    value={selectedObject.sensor.cameraName}
                    onChange={handleChange}
                />
            </label>
            <label>
                Horizontal FOV:
                <input
                    type="number"
                    name="horizontal_fov"
                    value={selectedObject.sensor.horizontal_fov}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Image Width:
                <input
                    type="number"
                    name="width"
                    value={selectedObject.sensor.width}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Height:
                <input
                    type="number"
                    name="height"
                    value={selectedObject.sensor.height}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Format:
                <input
                    type="text"
                    name="format"
                    value={selectedObject.sensor.format}
                    onChange={handleChange}
                />
            </label>
            <label>
                Clip Near:
                <input
                    type="number"
                    name="near"
                    value={selectedObject.sensor.near}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Clip Far:
                <input
                    type="number"
                    name="far"
                    value={selectedObject.sensor.far}
                    onChange={handleChange}
                />
                <span className="units">m</span>
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
                Image Topic Name:
                <input
                    type="text"
                    name="imageTopicName"
                    style={{ width: "200px" }}
                    value={selectedObject.sensor.imageTopicName}
                    onChange={handleChange}
                />
            </label>
            <label>
                Camera Info Topic Name:
                <input
                    type="text"
                    name="cameraInfoTopicName"
                    style={{ width: "200px" }}
                    value={selectedObject.sensor.cameraInfoTopicName}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default CameraParameters;
