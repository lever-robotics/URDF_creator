import React from 'react';

function CameraParameters({ userData, onChange }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        onChange({ ...userData, [name]: value });
    };

    return (
        <div>
            <label>
                Camera Name:
                <input
                    type="text"
                    name="cameraName"
                    value={userData.cameraName || 'camera'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Image Topic Name:
                <input
                    type="text"
                    name="imageTopicName"
                    value={userData.imageTopicName || '/camera/image_raw'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Camera Info Topic Name:
                <input
                    type="text"
                    name="cameraInfoTopicName"
                    value={userData.cameraInfoTopicName || '/camera/camera_info'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Horizontal FOV:
                <input
                    type="number"
                    name="horizontal_fov"
                    value={userData.horizontal_fov || 1.3962634}
                    onChange={handleChange}
                />
            </label>
            <label>
                Image Width:
                <input
                    type="number"
                    name="width"
                    value={userData.width || 800}
                    onChange={handleChange}
                />
            </label>
            <label>
                Image Height:
                <input
                    type="number"
                    name="height"
                    value={userData.height || 600}
                    onChange={handleChange}
                />
            </label>
            <label>
                Image Format:
                <input
                    type="text"
                    name="format"
                    value={userData.format || 'R8G8B8'}
                    onChange={handleChange}
                />
            </label>
            <label>
                Clip Near:
                <input
                    type="number"
                    name="near"
                    value={userData.near || 0.1}
                    onChange={handleChange}
                />
            </label>
            <label>
                Clip Far:
                <input
                    type="number"
                    name="far"
                    value={userData.far || 100}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default CameraParameters;
