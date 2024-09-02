import React, { useState, useEffect } from "react";
import CodeBox from "./CodeBox.jsx/CodeBox";
import "./RightPanel.css"; // Assuming you have a CSS file for this component
import ObjectParameters from "./ObjectParameters/ObjectParameters";

/**
 * @param {Scene} scene 
 * @param {string} projectTitle
 */
export default function RightPanel({ scene, projectTitle, selectedObject, stateFunctions, updateCode }) {
    const [selectedFormat, setSelectedFormat] = useState("Parameters");

    const handleClick = (e) => {
        const format = e.target.innerText;
        setSelectedFormat(format);
    };

    return (
        <div
            style={{
                display: "flex",
                flexDirection: "column",
                maxHeight: "100%",
                flexGrow: 1,
                flexBasis: 0,
            }}>
            <div className="toolbar">
                <button
                    className={
                        selectedFormat === "Parameters" ? "selected" : "toolbar-button"
                    }
                    onClick={handleClick}>
                    Parameters
                </button>
                <button
                    className={
                        selectedFormat === "URDF" ? "selected" : "toolbar-button"
                    }
                    onClick={handleClick}>
                    URDF
                </button>
                <button
                    className={
                        selectedFormat === "SDF" ? "selected" : "toolbar-button"
                    }
                    onClick={handleClick}>
                    SDF
                </button>
                <button
                    className={
                        selectedFormat === "XACRO" ? "selected" : "toolbar-button"
                    }
                    onClick={handleClick}>
                    XACRO
                </button>
            </div>
            <CodeBox
                scene={scene}
                projectTitle={projectTitle}
                selectedFormat={selectedFormat}
                updateCode={updateCode}
            />
            <ObjectParameters selectedObject={selectedObject} stateFunctions={stateFunctions} selectedFormat={selectedFormat}/>
        </div>
    );
}
