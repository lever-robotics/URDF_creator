import React, { useState, useEffect } from 'react';

function ScaleParameters({ selectedObject, onUpdate }) {
    const [scaleX, setScaleX] = useState('');
    const [scaleY, setScaleY] = useState('');
    const [scaleZ, setScaleZ] = useState('');

    useEffect(() => {
        console.log(selectedObject);
        if (selectedObject) {
            setScaleX(selectedObject.scale.x.toFixed(2));
            setScaleY(selectedObject.scale.y.toFixed(2));
            setScaleZ(selectedObject.scale.z.toFixed(2));
        }
    }, [JSON.stringify(selectedObject.scale)]);

    const handleChange = (axis, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const updatedObject = { ...selectedObject };

        if (selectedObject.userData.shape === 'sphere') {
            updatedObject.scale.x = newValue;
            updatedObject.scale.y = newValue;
            updatedObject.scale.z = newValue;
        } else if (selectedObject.userData.shape === 'cylinder') {
            if (axis === 'x' || axis === 'z') {
                updatedObject.scale.x = newValue;
                updatedObject.scale.z = newValue;
            } else {
                updatedObject.scale.y = newValue;
            }
        } else {
            updatedObject.scale[axis] = newValue;
        }

        onUpdate(updatedObject);
    };

    const handleBlur = (axis, value) => {
        handleChange(axis, value);
    };

    const handleKeyDown = (e, axis, value) => {
        if (e.key === 'Enter') {
            handleBlur(axis, value);
        }
    };

    let scaleInputs;
    switch (selectedObject.userData.shape) {
        case 'sphere':
            scaleInputs = (
                <li>
                    Radius:
                    <input
                        type="number"
                        value={scaleX}
                        onChange={(e) => setScaleX(e.target.value)}
                        onBlur={() => handleBlur('x', scaleX)}
                        onKeyDown={(e) => handleKeyDown(e, 'x', scaleX)}
                    />
                </li>
            );
            break;
        case 'cylinder':
            scaleInputs = (
                <>
                    <li>
                        Radius:
                        <input
                            type="number"
                            value={scaleX}
                            onChange={(e) => setScaleX(e.target.value)}
                            onBlur={() => handleBlur('x', scaleX)}
                            onKeyDown={(e) => handleKeyDown(e, 'x', scaleX)}
                        />
                    </li>
                    <li>
                        Height:
                        <input
                            type="number"
                            value={scaleY}
                            onChange={(e) => setScaleY(e.target.value)}
                            onBlur={() => handleBlur('y', scaleY)}
                            onKeyDown={(e) => handleKeyDown(e, 'y', scaleY)}
                        />
                    </li>
                </>
            );
            break;
        default:
            scaleInputs = (
                <>
                    <li>
                        X:
                        <input
                            type="number"
                            value={scaleX}
                            onChange={(e) => setScaleX(e.target.value)}
                            onBlur={() => handleBlur('x', scaleX)}
                            onKeyDown={(e) => handleKeyDown(e, 'x', scaleX)}
                        />
                    </li>
                    <li>
                        Y:
                        <input
                            type="number"
                            value={scaleY}
                            onChange={(e) => setScaleY(e.target.value)}
                            onBlur={() => handleBlur('y', scaleY)}
                            onKeyDown={(e) => handleKeyDown(e, 'y', scaleY)}
                        />
                    </li>
                    <li>
                        Z:
                        <input
                            type="number"
                            value={scaleZ}
                            onChange={(e) => setScaleZ(e.target.value)}
                            onBlur={() => handleBlur('z', scaleZ)}
                            onKeyDown={(e) => handleKeyDown(e, 'z', scaleZ)}
                        />
                    </li>
                </>
            );
            break;
    }

    return (
        <ul>
            {scaleInputs}
        </ul>
    );
}

export default ScaleParameters;
