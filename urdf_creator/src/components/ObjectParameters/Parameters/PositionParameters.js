import React, { useState, useEffect } from 'react';

function PositionParameters({ selectedObject, onUpdate }) {
    const [positionX, setPositionX] = useState('');
    const [positionY, setPositionY] = useState('');
    const [positionZ, setPositionZ] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setPositionX(selectedObject.position.x.toFixed(2));
            setPositionY(selectedObject.position.y.toFixed(2));
            setPositionZ(selectedObject.position.z.toFixed(2));
        }
    }, [JSON.stringify(selectedObject.position)]);

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
                    value={positionX}
                    onChange={(e) => setPositionX(e.target.value)}
                    onBlur={() => handleBlur('position', 'x', positionX)}
                    onKeyDown={(e) => handleKeyDown(e, 'position', 'x', positionX)}
                />
            </li>
            <li>
                Y:
                <input
                    type="number"
                    value={positionY}
                    onChange={(e) => setPositionY(e.target.value)}
                    onBlur={() => handleBlur('position', 'y', positionY)}
                    onKeyDown={(e) => handleKeyDown(e, 'position', 'y', positionY)}
                />
            </li>
            <li>
                Z:
                <input
                    type="number"
                    value={positionZ}
                    onChange={(e) => setPositionZ(e.target.value)}
                    onBlur={() => handleBlur('position', 'z', positionZ)}
                    onKeyDown={(e) => handleKeyDown(e, 'position', 'z', positionZ)}
                />
            </li>
        </ul>
    );
}

export default PositionParameters;
