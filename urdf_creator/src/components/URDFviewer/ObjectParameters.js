import React, { useContext } from 'react';
import { URDFContext } from '../URDFContext/URDFContext';

function ObjectParameters() {
    const { selectedObject, setSelectedObject } = useContext(URDFContext);

    if (!selectedObject) {
        return <div style={{ padding: 10 }}>No object selected</div>;
    }

    const handleChange = (e) => {
        const { name, value } = e.target;
        setSelectedObject({ ...selectedObject, [name]: parseFloat(value) });
    };

    return (
        <div style={{ width: '300px', padding: 10 }}>
            <h3>Object Parameters</h3>
            <div>
                <label>X Position</label>
                <input
                    type="number"
                    name="x"
                    value={selectedObject.position.x}
                    onChange={handleChange}
                />
            </div>
            <div>
                <label>Y Position</label>
                <input
                    type="number"
                    name="y"
                    value={selectedObject.position.y}
                    onChange={handleChange}
                />
            </div>
            <div>
                <label>Z Position</label>
                <input
                    type="number"
                    name="z"
                    value={selectedObject.position.z}
                    onChange={handleChange}
                />
            </div>
            {/* Add more parameters as needed */}
        </div>
    );
}

export default ObjectParameters;
