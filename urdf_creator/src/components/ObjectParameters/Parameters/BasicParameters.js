import React, { useState } from "react";
import "./parameters_style.css";
import Parameter from "./Parameter";
import ToggleSection from "../ToggleSection";

export default function BasicParameters({ selectedObject }) {
    const [name, changeName] = useState(selectedObject.name);
    const [color, setColor] = useState("#" + selectedObject.color.getHexString());
    const [error, setError] = useState(""); // Step 5: Add error state

    if(name !== selectedObject.name){
        setColor("#" + selectedObject.color.getHexString());
        changeName(selectedObject.name);
    }

    const handleNameChange = (e) => {
        const newName = e.target.value;
        if (newName.includes(" ")) {
            // Check for spaces
            changeName(newName);
            setError("Name must have no spaces"); // Set error message
        } else {
            changeName(newName);
            selectedObject.name(newName);
            setError(""); // Clear error message if input is valid
        }
    };

    const handleColorChange = (e) => {
        selectedObject.color = e.target.value;
        setColor(e.target.value);
    };

    return (
        <ToggleSection title="Basic Parameters">
            <ul>
                <Parameter
                    title={"Name:"}
                    type={"text"}
                    value={name}
                    onChange={handleNameChange}
                    readOnly={selectedObject.name === "base_link"}
                    className={"name-input"}
                />
                <Parameter
                    title={"Color:"}
                    type="color"
                    value={color}
                    onChange={handleColorChange}
                />
            </ul>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
        </ToggleSection>
    );
}
