import React, { useState } from "react";
import CodeBox from "./CodeBox/CodeBox";
import "./RightPanel.css"; // Assuming you have a CSS file for this component
import ObjectParameters from "./ObjectParameters/ObjectParameters";
import Frame, { Frameish } from "../../Models/Frame";
import { StateFunctionsType } from "../SceneState";
import { Scene } from "three";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 */

type Props = {
    scene: Scene,
    projectTitle: string,
    selectedObject?: Frameish,
    stateFunctions: StateFunctionsType,
    updateCode: number,
    className: string
}

export default function RightPanel({ scene, projectTitle, selectedObject, stateFunctions, updateCode }: Props) {
    const [selectedFormat, setSelectedFormat] = useState("Parameters");

    const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        const format = e.currentTarget.innerText;
        setSelectedFormat(format);
    };

    return (
        <div className="right-panel">
            <div className="toolbar">
                <button className={selectedFormat === "Parameters" ? "selected" : "toolbar-button"} onClick={handleClick}>
                    Parameters
                </button>
                <button className={selectedFormat === "URDF" ? "selected" : "toolbar-button"} onClick={handleClick}>
                    URDF
                </button>
                <button className={selectedFormat === "SDF" ? "selected" : "toolbar-button"} onClick={handleClick}>
                    SDF
                </button>
                <button className={selectedFormat === "XACRO" ? "selected" : "toolbar-button"} onClick={handleClick}>
                    XACRO
                </button>
            </div>
            <CodeBox scene={scene} projectTitle={projectTitle} selectedFormat={selectedFormat} updateCode={updateCode} />
            <ObjectParameters selectedObject={selectedObject} stateFunctions={stateFunctions} selectedFormat={selectedFormat} />
        </div>
    );
}
