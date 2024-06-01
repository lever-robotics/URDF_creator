import React from 'react';

function IMUParameters({ userData, onChange }) {
    const handleChange = (e) => {
        const { name, value } = e.target;
        onChange({ ...userData, [name]: value });
    };

    return (
        <div>
            <label>
                Gaussian Noise:
                <input
                    type="number"
                    name="gaussianNoise"
                    value={userData.gaussianNoise || 0}
                    onChange={handleChange}
                />
            </label>
            <label>
                XYZ Offsets:
                <input
                    type="text"
                    name="xyzOffsets"
                    value={userData.xyzOffsets || '0 0 0'}
                    onChange={handleChange}
                />
            </label>
            <label>
                RPY Offsets:
                <input
                    type="text"
                    name="rpyOffsets"
                    value={userData.rpyOffsets || '0 0 0'}
                    onChange={handleChange}
                />
            </label>
        </div>
    );
}

export default IMUParameters;
