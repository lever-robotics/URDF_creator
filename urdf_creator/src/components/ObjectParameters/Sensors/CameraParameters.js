import React from 'react';
import { Camera } from '../../../Models/SensorsClass';

function CameraParameters({ selectedObject, sensorData, setSensor }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        setSensor(selectedObject, new Camera({ ...sensorData, [name]: value }));
    };

    return (
        <div>
            <label>
                Camera Name:
                <input
                    type="text"
                    name="cameraName"
                    value={sensorData.cameraName || 'camera'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Horizontal FOV:
                <input
                    type="number"
                    name="horizontal_fov"
                    value={sensorData.horizontal_fov || 1.3962634}
                    onChange={handleChange}
                />
                <span className="units">&deg; degrees</span>
            </label>
            <label>
                Image Width:
                <input
                    type="number"
                    name="width"
                    value={sensorData.width || 800}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Height:
                <input
                    type="number"
                    name="height"
                    value={sensorData.height || 600}
                    onChange={handleChange}
                />
                <span className="units">px</span>
            </label>
            <label>
                Image Format:
                <input
                    type="text"
                    name="format"
                    value={sensorData.format || 'R8G8B8'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Clip Near:
                <input
                    type="number"
                    name="near"
                    value={sensorData.near || 0.1}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Clip Far:
                <input
                    type="number"
                    name="far"
                    value={sensorData.far || 100}
                    onChange={handleChange}
                />
                <span className="units">m</span>
            </label>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={sensorData.gaussianNoise || 0.01}
                    onChange={handleChange}
                />
            </label>
            <label>
                Update Rate:
                <input
                    type="number"
                    name="updateRate"
                    value={sensorData.updateRate || 100}
                    onChange={handleChange}
                />
                <span className="units">Hz</span>
            </label>
            <label>
                Image Topic Name:
                <input
                    type="text"
                    name="imageTopicName"
                    style={{ width: '200px' }}
                    value={sensorData.imageTopicName || '/camera/image_raw'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Camera Info Topic Name:
                <input
                    type="text"
                    name="cameraInfoTopicName"
                    style={{ width: '200px' }}
                    value={sensorData.cameraInfoTopicName || '/camera/camera_info'}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default CameraParameters;
