import React, { useState, useEffect } from 'react';

function JointParameters({ selectedObject, onUpdate }) {
    const [jointType, setJointType] = useState('');

    useEffect(() => {
        if (selectedObject) {
            setJointType(selectedObject.userData.jointType || '');
        }
    }, [JSON.stringify(selectedObject.userData)]);

    const handleJointTypeChange = (e) => {
        setJointType(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.jointType = e.target.value;
        onUpdate(updatedObject);
    };

    return (
        <div>
            <strong>Joint Information:</strong>
            <div>
                <strong>Parent Link:</strong>
                <span>{selectedObject.parent.userData.name}</span>
            </div>
            <div>
                <strong>Joint Type:</strong>
                <select value={jointType} onChange={handleJointTypeChange}>
                    <option value="">Select a joint type</option>
                    <option value="fixed">Fixed</option>
                    <option value="revolute">Revolute</option>
                    <option value="continuous">Continuous</option>
                    <option value="prismatic">Prismatic</option>
                    <option value="planar">Planar</option>
                    <option value="floating">Floating</option>
                </select>
            </div>
        </div>
    );
}

export default JointParameters;
