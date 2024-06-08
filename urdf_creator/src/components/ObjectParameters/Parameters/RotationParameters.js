import React, { useState, useEffect } from 'react';

function RotationParameters({ selectedObject, transformObject }) {
    const [rotationX, setRotationX] = useState('');
    const [rotationY, setRotationY] = useState('');
    const [rotationZ, setRotationZ] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setRotationX(radToDeg(selectedObject.rotation.x).toFixed(2));
            setRotationY(radToDeg(selectedObject.rotation.y).toFixed(2));
            setRotationZ(radToDeg(selectedObject.rotation.z).toFixed(2));
        }
    }, [JSON.stringify(selectedObject.rotation)]);

    const radToDeg = (radians) => (radians * 180) / Math.PI;
    const degToRad = (degrees) => (degrees * Math.PI) / 180;

    const handleChange = (prop, axis, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        transformObject(selectedObject, 'rotation', degToRad(rotationX), degToRad(rotationY), degToRad(rotationZ));
    };

    const handleBlur = (prop, axis, value) => {
        handleChange(prop, axis, value);
    };

    const handleKeyDown = (e, prop, axis, value) => {
        if (e.key === 'Enter') {
            handleBlur(prop, axis, value);
        }
    };

    return (
        <ul>
            <li>
                X:
                <input
                    type="number"
                    value={rotationX}
                    onChange={(e) => setRotationX(e.target.value)}
                    onBlur={() => handleBlur('rotation', 'x', rotationX)}
                    onKeyDown={(e) => handleKeyDown(e, 'rotation', 'x', rotationX)}
                />
                <span className="units">&deg; degrees</span>
            </li>
            <li>
                Y:
                <input
                    type="number"
                    value={rotationY}
                    onChange={(e) => setRotationY(e.target.value)}
                    onBlur={() => handleBlur('rotation', 'y', rotationY)}
                    onKeyDown={(e) => handleKeyDown(e, 'rotation', 'y', rotationY)}
                />
                <span className="units">&deg; degrees</span>
            </li>
            <li>
                Z:
                <input
                    type="number"
                    value={rotationZ}
                    onChange={(e) => setRotationZ(e.target.value)}
                    onBlur={() => handleBlur('rotation', 'z', rotationZ)}
                    onKeyDown={(e) => handleKeyDown(e, 'rotation', 'z', rotationZ)}
                />
                <span className="units">&deg; degrees</span>
            </li>
        </ul>
    );
}

export default RotationParameters;
