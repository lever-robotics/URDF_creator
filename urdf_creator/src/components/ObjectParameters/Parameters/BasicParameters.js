import React, { useState, useEffect } from "react";
import * as THREE from "three";

export default function BasicParameters({ selectedObject, setLinkName, setUserColor, setMass }) {
    const [name, setName] = useState("");
    const [mass, setMassTemp] = useState("");
    const [color, setColor] = useState("#ffffff");

    useEffect(() => {
        if (selectedObject) {
            setName(selectedObject.userData.name || "");
            setMassTemp(selectedObject.userData.inertia.mass || "");
            setColor(new THREE.Color(selectedObject.mesh.material.color).getStyle());
        }
    }, [JSON.stringify(selectedObject.userData), JSON.stringify(selectedObject.mesh.material)]);

    const handleNameChange = (e) => {
        setName(e.target.value);
        setLinkName(selectedObject, e.target.value);
    };

    const handleMassChange = (e) => {
        setMassTemp(e.target.value);
        setMass(selectedObject, parseInt(e.target.value, 10));
    };

    const handleColorChange = (e) => {
        setColor(e.target.value);
        setUserColor(selectedObject, e.target.value);
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
