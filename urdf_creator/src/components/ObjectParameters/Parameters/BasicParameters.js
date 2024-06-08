import React, { useState, useEffect } from "react";
import * as THREE from "three";
import './parameters_style.css';

export default function BasicParameters({ selectedObject, setLinkName, setUserColor }) {
    const [name, setName] = useState("");
    const [color, setColor] = useState("#ffffff");

    useEffect(() => {
        if (selectedObject) {
            setName(selectedObject.userData.name || "");
            setColor(new THREE.Color(selectedObject.mesh.material.color).getStyle());
        }
    }, [JSON.stringify(selectedObject.userData), JSON.stringify(selectedObject.mesh.material)]);

    const handleNameChange = (e) => {
        setName(e.target.value);
        setLinkName(selectedObject, e.target.value);
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
                onChange={handleNameChange}
                readOnly={selectedObject.userData.name === "base_link"}
                title={selectedObject.userData.name === "base_link" ? "The base_link's name is not changeable." : ""}
            />
            <br />
            <br />
            <strong>Color:</strong>
            <input type="color" value={color} onChange={handleColorChange} style={{ borderColor: color }} />
        </div>
    );
}
