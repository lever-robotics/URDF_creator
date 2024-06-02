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
                Image Topic Name:
                <input
                    type="text"
                    name="imageTopicName"
                    value={sensorData.imageTopicName || '/camera/image_raw'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Camera Info Topic Name:
                <input
                    type="text"
                    name="cameraInfoTopicName"
                    value={sensorData.cameraInfoTopicName || '/camera/camera_info'}
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
            </label>
            <label>
                Image Width:
                <input
                    type="number"
                    name="width"
                    value={sensorData.width || 800}
                    onChange={handleChange}
                />
            </label>
            <label>
                Image Height:
                <input
                    type="number"
                    name="height"
                    value={sensorData.height || 600}
                    onChange={handleChange}
                />
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
            </label>
            <label>
                Clip Far:
                <input
                    type="number"
                    name="far"
                    value={sensorData.far || 100}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default CameraParameters;
