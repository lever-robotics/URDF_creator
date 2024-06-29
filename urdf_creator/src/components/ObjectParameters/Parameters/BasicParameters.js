import React, { useState, useEffect } from "react";
import * as THREE from "three";
import './parameters_style.css';

export default function BasicParameters({ selectedObject, setUserColor }) {
    const [name, setName] = useState("");
    const [color, setColor] = useState("#ffffff");
    const [error, setError] = useState(""); // Step 5: Add error state

    useEffect(() => {
        if (selectedObject) {
            setName(selectedObject.userData.name || "");
            setColor(new THREE.Color(selectedObject.mesh.material.color).getStyle());
        }
    }, [JSON.stringify(selectedObject.userData), JSON.stringify(selectedObject.mesh.material)]);

    const handleNameChange = (e) => {
        const newName = e.target.value;
        if (newName.includes(" ")) { // Step 2: Check for spaces
            setName(newName);
            setError("Name must have no spaces"); // Step 3: Set error message
        } else {
            setName(newName);
            selectedObject.setName(newName);
            setError(""); // Clear error message if input is valid
        }
    };

    const handleColorChange = (e) => {
        setColor(e.target.value);
        setUserColor(selectedObject, e.target.value);
    };

    return (
        <div>
            <strong>Name:</strong>
            <input
                className="name-input"
                type="text"
                value={name}
                style={{ width: '100px' }}
                onChange={handleNameChange}
                readOnly={selectedObject.userData.name === "base_link"}
                title={selectedObject.userData.name === "base_link" ? "The base_link's name is not changeable." : ""}
            />
            {error && <span style={{ color: 'red', marginLeft: '5px' }}>{error}</span>} {/* Step 6: Display error message */}
            <br />
            <br />
            <strong>Color:</strong>
            <input type="color" value={color} onChange={handleColorChange} style={{ borderColor: color }} />
        </div>
    );
}