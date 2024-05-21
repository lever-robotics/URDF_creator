import React, { useState, useEffect } from 'react';
import './ObjectParameters.css';
import * as THREE from 'three';
import ToggleSection from './ToggleSection';

function ObjectParameters({ selectedObject, onUpdate }) {
    const [isSensor, setIsSensor] = useState(false);
    const [sensorType, setSensorType] = useState("");
    const [isBaseLink, setIsBaseLink] = useState(false);

    useEffect(() => {
        if (selectedObject) {
            setIsBaseLink(selectedObject.userData?.isBaseLink || false);
        }
    }, [selectedObject]);

    if (!selectedObject) return <div></div>;

    const { position, rotation, scale, userData, parent } = selectedObject;
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

    const handleSensorTypeChange = (e) => {
        setSensorType(e.target.value);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.sensorType = e.target.value;
        onUpdate(updatedObject);
    };

    const handleJointTypeChange = (e) => {
        const updatedObject = { ...selectedObject };
        updatedObject.userData.jointType = e.target.value;
        onUpdate(updatedObject);
    };

    const handleBaseLinkChange = (e) => {
        setIsBaseLink(e.target.checked);
        const updatedObject = { ...selectedObject };
        updatedObject.userData.isBaseLink = e.target.checked;
        updatedObject.userData.name = e.target.checked ? 'base_link' : userData.name;
        onUpdate(updatedObject);
    };

    return (
        <div className="object-parameters">
            <h3>Object Parameters</h3>
            <ToggleSection title="Basic Parameters">
                <div>
                    <strong>Name:</strong>
                    <input type="text" value={isBaseLink ? "base_link" : (userData.name || "")} onChange={handleNameChange} disabled={isBaseLink} />
                </div>
                <div>
                    <strong>Mass:</strong>
                    <input type="number" value={userData.mass || ""} onChange={handleMassChange} />
                </div>
                <div>
                    <strong>Color:</strong>
                    <input type="color" value={color} onChange={handleColorChange} style={{ borderColor: color }} />
                </div>
            </ToggleSection>
            <ToggleSection title="Position">
                <ul>
                    <li>X: <input type="number" value={position.x.toFixed(2)} onChange={(e) => handleChange('position', 'x', e.target.value)} /></li>
                    <li>Y: <input type="number" value={position.y.toFixed(2)} onChange={(e) => handleChange('position', 'y', e.target.value)} /></li>
                    <li>Z: <input type="number" value={position.z.toFixed(2)} onChange={(e) => handleChange('position', 'z', e.target.value)} /></li>
                </ul>
            </ToggleSection>
            <ToggleSection title="Rotation">
                <ul>
                    <li>X: <input type="number" value={radToDeg(rotation.x).toFixed(2)} onChange={(e) => handleChange('rotation', 'x', e.target.value)} /></li>
                    <li>Y: <input type="number" value={radToDeg(rotation.y).toFixed(2)} onChange={(e) => handleChange('rotation', 'y', e.target.value)} /></li>
                    <li>Z: <input type="number" value={radToDeg(rotation.z).toFixed(2)} onChange={(e) => handleChange('rotation', 'z', e.target.value)} /></li>
                </ul>
            </ToggleSection>
            <ToggleSection title="Scale">
                <ul>
                    <li>X: <input type="number" value={scale.x.toFixed(2)} onChange={(e) => handleChange('scale', 'x', e.target.value)} /></li>
                    <li>Y: <input type="number" value={scale.y.toFixed(2)} onChange={(e) => handleChange('scale', 'y', e.target.value)} /></li>
                    <li>Z: <input type="number" value={scale.z.toFixed(2)} onChange={(e) => handleChange('scale', 'z', e.target.value)} /></li>
                </ul>
            </ToggleSection>
            <ToggleSection title="Additional Parameters">
                <div>
                    <strong>Density:</strong>
                    <input type="number" value={userData.density || ""} onChange={(e) => handleAdditionalChange('density', e.target.value)} />
                </div>
                <div>
                    <strong>Moment of Inertia:</strong>
                    <ul>
                        <li>Ixx: <input type="number" value={userData.Ixx || ""} onChange={(e) => handleAdditionalChange('Ixx', e.target.value)} /></li>
                        <li>Ixy: <input type="number" value={userData.Ixy || ""} onChange={(e) => handleAdditionalChange('Ixy', e.target.value)} /></li>
                        <li>Ixz: <input type="number" value={userData.Ixz || ""} onChange={(e) => handleAdditionalChange('Ixz', e.target.value)} /></li>
                        <li>Iyy: <input type="number" value={userData.Iyy || ""} onChange={(e) => handleAdditionalChange('Iyy', e.target.value)} /></li>
                        <li>Iyz: <input type="number" value={userData.Iyz || ""} onChange={(e) => handleAdditionalChange('Iyz', e.target.value)} /></li>
                        <li>Izz: <input type="number" value={userData.Izz || ""} onChange={(e) => handleAdditionalChange('Izz', e.target.value)} /></li>
                    </ul>
                </div>
                <div>
                    <strong>Friction:</strong>
                    <input type="number" value={userData.friction || ""} onChange={(e) => handleAdditionalChange('friction', e.target.value)} />
                </div>
            </ToggleSection>
            <div>
                <label>
                    <input
                        type="checkbox"
                        checked={isSensor}
                        onChange={() => setIsSensor(!isSensor)}
                    />
                    Is Sensor
                </label>
                {isSensor && (
                    <div>
                        <strong>Sensor Type:</strong>
                        <select value={sensorType} onChange={handleSensorTypeChange}>
                            <option value="">Select a sensor</option>
                            <option value="lidar">Lidar</option>
                            <option value="camera">Camera</option>
                            <option value="imu">IMU</option>
                            <option value="gps">GPS</option>
                        </select>
                    </div>
                )}
                {!userData.isBaseLink && (
                    <div>
                        <strong>Joint Information:</strong>
                        <div>
                            <strong>Parent Link:</strong>
                            <span>{parent.userData.name}</span>
                        </div>
                        <div>
                            <strong>Joint Type:</strong>
                            <select value={userData.jointType || ""} onChange={handleJointTypeChange}>
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
                )}
            </div>
        </div>
    );
}

export default ObjectParameters;
