import React, { useState } from "react";
import "./parameters_style.css";
import Parameter from "./Parameter";
import ToggleSection from "../ToggleSection";

export default function BasicParameters({ stateFunctions, selectedObject }) {
    const [error, setError] = useState("");

    const handleNameChange = (e) => {
        const newName = e.target.value;
        if (newName.includes(" ")) {
            setError("Name must have no spaces");
        } else {
            stateFunctions.setLinkName(selectedObject, newName);
            setError("");
        }
    };

    const handleColorChange = (e) => {
        stateFunctions.setLinkColor(selectedObject, e.target.value);
    };

    return (
        <ToggleSection title="Basic Parameters">
            <ul>
                <Parameter
                    title={"Name:"}
                    type={"text"}
                    value={selectedObject.name}
                    onChange={handleNameChange}
                    readOnly={selectedObject.name === "base_link"}
                    className={"name-input"}
                />
                <Parameter
                    title={"Color:"}
                    type="color"
                    value={"#"+ selectedObject.color.getHexString()}
                    onChange={handleColorChange}
                />
            </ul>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
        </ToggleSection>
    );
}
