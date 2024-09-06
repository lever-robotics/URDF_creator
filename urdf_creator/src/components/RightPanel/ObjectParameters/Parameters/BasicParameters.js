import React, { useState, useEffect } from "react";
import "./parameters_style.css";
import "../ObjectParameters.css";
import Parameter from "./Parameter";
import ToggleSection from "../ToggleSection";
import Section from "../Section";
import PositionParameters from "./PositionParameters";
import RotationParameters from "./RotationParameters";
import ScaleParameters from "./ScaleParameters";

export default function BasicParameters({ stateFunctions, selectedObject }) {
    const [error, setError] = useState("");
    const [tempName, setTempName] = useState(selectedObject.name);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempName(selectedObject.name);
        setError("");
    }, [JSON.stringify(selectedObject.name)]);

    const handleNameChange = (e) => {
        const newName = e.target.value;
        if (newName.includes(" ")) {
            setError("Name must have no spaces");
        } else {
            setTempName(newName);
        }
    };

    const handleNameBlur = (e) => {
        const newName = e.target.value;
        if(newName === selectedObject.name){
            setError("");
        }else if(stateFunctions.doesLinkNameExist(newName)){
            setError("Name must be unique");
        }else if (newName === "") {
            setError("Name cannot be empty");
        } else {
            stateFunctions.setLinkName(selectedObject, newName);
            setError("");
        }
    }

    const handleKeyDown = (e) => {
        if(e.key === "Enter"){
            handleNameBlur(e);
        }
    }

    const handleColorChange = (e) => {
        stateFunctions.setLinkColor(selectedObject, e.target.value);
    };

    const handleColorBlur = (e) => {
        stateFunctions.forceSceneUpdate();
    };

    return (
        <Section title="Basic Parameters">
            <ul>
                <Parameter
                    title={"Name:"}
                    type={"text"}
                    value={tempName}
                    onChange={handleNameChange}
                    onBlur={handleNameBlur}
                    onKeyDown={handleKeyDown}
                    readOnly={selectedObject.name === "base_link"}
                    className={"name-input"}
                />
                <Parameter
                    title={"Color:"}
                    type="color"
                    value={"#"+ selectedObject.color.getHexString()}
                    onChange={handleColorChange}
                    onBlur={handleColorBlur}
                />
            </ul>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
        </Section>
    );
}
