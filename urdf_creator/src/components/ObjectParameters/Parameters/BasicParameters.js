import React, { useState, useEffect } from 'react';
import * as THREE from 'three';

function BasicParameters({ selectedObject, onUpdate }) {
    const [name, setName] = useState('');
    const [mass, setMass] = useState('');
    const [color, setColor] = useState('#ffffff');

    useEffect(() => {
        if (selectedObject) {
            setName(selectedObject.userData.name || '');
            setMass(selectedObject.userData.mass || '');
            setColor(new THREE.Color(selectedObject.material.color).getStyle());
        }
    }, [JSON.stringify(selectedObject.userData), JSON.stringify(selectedObject.material)]);

    const handleNameChange = (e) => {
        setName(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.name = e.target.value;
        onUpdate(updatedObject);
    };

    const handleMassChange = (e) => {
        setMass(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.mass = parseFloat(e.target.value) || 0;
        onUpdate(updatedObject);
    };

    const handleColorChange = (e) => {
        setColor(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.material.color = new THREE.Color(e.target.value);
        onUpdate(updatedObject);
    };

    return (
        <div>
            <strong>Name:</strong>
            <input type="text" value={name} onChange={handleNameChange} />
            <strong>Mass:</strong>
            <input type="number" value={mass} onChange={handleMassChange} />
            <strong>Color:</strong>
            <input type="color" value={color} onChange={handleColorChange} style={{ borderColor: color }} />
        </div>
    );
}

export default BasicParameters;
