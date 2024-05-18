import React, { useState } from 'react';
import './ObjectParameters.css';
import * as THREE from 'three';

function ObjectParameters({ selectedObject, onUpdate }) {
    const [showAdditional, setShowAdditional] = useState(false);

    if (!selectedObject) return <div></div>;

    const { position, rotation, scale, userData } = selectedObject;

    // Get the color of the selected object
    const color = new THREE.Color(selectedObject.material.color).getStyle();

    const radToDeg = (radians) => (radians * 180) / Math.PI;
    const degToRad = (degrees) => (degrees * Math.PI) / 180;

    const handleChange = (prop, axis, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const updatedObject = { ...selectedObject };
        updatedObject[prop][axis] = prop === 'rotation' ? degToRad(newValue) : newValue;
        onUpdate(updatedObject);
    };

    const handleNameChange = (e) => {
        const updatedObject = { ...selectedObject };
        updatedObject.userData.name = e.target.value;
        onUpdate(updatedObject);
    };

    const handleMassChange = (e) => {
        const updatedObject = { ...selectedObject };
        updatedObject.userData.mass = parseFloat(e.target.value) || 0;
        onUpdate(updatedObject);
    };

    const handleColorChange = (e) => {
        const updatedObject = { ...selectedObject };
        updatedObject.material.color = new THREE.Color(e.target.value);
        onUpdate(updatedObject);
    };

    const handleAdditionalChange = (prop, value) => {
        const updatedObject = { ...selectedObject };
        updatedObject.userData[prop] = parseFloat(value) || 0;
        onUpdate(updatedObject);
    };

    return (
        <div className="object-parameters">
            <h3>Object Parameters</h3>
            <div>
                <strong>Name:</strong>
                <input type="text" value={userData.name || ""} onChange={handleNameChange} style={{ borderColor: color }} />
            </div>
            <div>
                <strong>Mass:</strong>
                <input type="number" value={userData.mass || ""} onChange={handleMassChange} style={{ borderColor: color }} />
            </div>
            <div>
                <strong>Color:</strong>
                <input type="color" value={color} onChange={handleColorChange} style={{ borderColor: color }} />
            </div>
            <div>
                <strong>Position:</strong>
                <ul>
                    <li>X: <input type="number" value={position.x.toFixed(2)} onChange={(e) => handleChange('position', 'x', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Y: <input type="number" value={position.y.toFixed(2)} onChange={(e) => handleChange('position', 'y', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Z: <input type="number" value={position.z.toFixed(2)} onChange={(e) => handleChange('position', 'z', e.target.value)} style={{ borderColor: color }} /></li>
                </ul>
            </div>
            <div>
                <strong>Rotation:</strong>
                <ul>
                    <li>X: <input type="number" value={radToDeg(rotation.x).toFixed(2)} onChange={(e) => handleChange('rotation', 'x', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Y: <input type="number" value={radToDeg(rotation.y).toFixed(2)} onChange={(e) => handleChange('rotation', 'y', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Z: <input type="number" value={radToDeg(rotation.z).toFixed(2)} onChange={(e) => handleChange('rotation', 'z', e.target.value)} style={{ borderColor: color }} /></li>
                </ul>
            </div>
            <div>
                <strong>Scale:</strong>
                <ul>
                    <li>X: <input type="number" value={scale.x.toFixed(2)} onChange={(e) => handleChange('scale', 'x', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Y: <input type="number" value={scale.y.toFixed(2)} onChange={(e) => handleChange('scale', 'y', e.target.value)} style={{ borderColor: color }} /></li>
                    <li>Z: <input type="number" value={scale.z.toFixed(2)} onChange={(e) => handleChange('scale', 'z', e.target.value)} style={{ borderColor: color }} /></li>
                </ul>
            </div>
            <div>
                <label>
                    <input type="checkbox" checked={showAdditional} onChange={() => setShowAdditional(!showAdditional)} />
                    Show Additional Parameters
                </label>
            </div>
            {showAdditional && (
                <div>
                    <strong>Density:</strong>
                    <input type="number" value={userData.density || ""} onChange={(e) => handleAdditionalChange('density', e.target.value)} style={{ borderColor: color }} />
                    <div>
                        <strong>Moment of Inertia:</strong>
                        <ul>
                            <li>Ixx: <input type="number" value={userData.Ixx || ""} onChange={(e) => handleAdditionalChange('Ixx', e.target.value)} style={{ borderColor: color }} /></li>
                            <li>Ixy: <input type="number" value={userData.Ixy || ""} onChange={(e) => handleAdditionalChange('Ixy', e.target.value)} style={{ borderColor: color }} /></li>
                            <li>Ixz: <input type="number" value={userData.Ixz || ""} onChange={(e) => handleAdditionalChange('Ixz', e.target.value)} style={{ borderColor: color }} /></li>
                            <li>Iyy: <input type="number" value={userData.Iyy || ""} onChange={(e) => handleAdditionalChange('Iyy', e.target.value)} style={{ borderColor: color }} /></li>
                            <li>Iyz: <input type="number" value={userData.Iyz || ""} onChange={(e) => handleAdditionalChange('Iyz', e.target.value)} style={{ borderColor: color }} /></li>
                            <li>Izz: <input type="number" value={userData.Izz || ""} onChange={(e) => handleAdditionalChange('Izz', e.target.value)} style={{ borderColor: color }} /></li>
                        </ul>
                    </div>
                    <div>
                        <strong>Friction:</strong>
                        <input type="number" value={userData.friction || ""} onChange={(e) => handleAdditionalChange('friction', e.target.value)} style={{ borderColor: color }} />
                    </div>
                </div>
            )}
        </div>
    );
}

export default ObjectParameters;
