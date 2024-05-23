import React, { useState, useEffect } from 'react';

function ScaleParameters({ selectedObject, onUpdate }) {
    const [scaleX, setScaleX] = useState('');
    const [scaleY, setScaleY] = useState('');
    const [scaleZ, setScaleZ] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setScaleX(selectedObject.scale.x.toFixed(2));
            setScaleY(selectedObject.scale.y.toFixed(2));
            setScaleZ(selectedObject.scale.z.toFixed(2));
        }
    }, [selectedObject]);

    const handleChange = (prop, axis, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const updatedObject = { ...selectedObject };
        updatedObject[prop][axis] = newValue;
        onUpdate(updatedObject);
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
                    value={scaleX}
                    onChange={(e) => setScaleX(e.target.value)}
                    onBlur={() => handleBlur('scale', 'x', scaleX)}
                    onKeyDown={(e) => handleKeyDown(e, 'scale', 'x', scaleX)}
                />
            </li>
            <li>
                Y:
                <input
                    type="number"
                    value={scaleY}
                    onChange={(e) => setScaleY(e.target.value)}
                    onBlur={() => handleBlur('scale', 'y', scaleY)}
                    onKeyDown={(e) => handleKeyDown(e, 'scale', 'y', scaleY)}
                />
            </li>
            <li>
                Z:
                <input
                    type="number"
                    value={scaleZ}
                    onChange={(e) => setScaleZ(e.target.value)}
                    onBlur={() => handleBlur('scale', 'z', scaleZ)}
                    onKeyDown={(e) => handleKeyDown(e, 'scale', 'z', scaleZ)}
                />
            </li>
        </ul>
    );
}

export default ScaleParameters;
