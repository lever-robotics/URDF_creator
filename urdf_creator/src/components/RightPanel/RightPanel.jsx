import React, { useState, useEffect } from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import ScenetoText from "../../utils/ScenetoText";
import "./RightPanel.css"; // Assuming you have a CSS file for this component
import ObjectParameters from "./ObjectParameters/ObjectParameters";
import Tooltip from "../../FunctionalComponents/Tooltip";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 */
export default function RightPanel({ scene, projectTitle, selectedObject, stateFunctions }) {
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
            }}
        >
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
            <CodeBox scene={scene} projectTitle={projectTitle} selectedFormat={selectedFormat} />
            <ObjectParameters selectedObject={selectedObject} stateFunctions={stateFunctions} selectedFormat={selectedFormat} />
        </div>
    );
}

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 * @param {string} selectedFormat
 */
function CodeBox({ scene, projectTitle, selectedFormat }) {
    const style = {
        fontSize: "12px",
        overflow: "auto",
        height: "100%",
        width: "100%",
        flexGrow: 1,
    };
    const [copied, setCopied] = useState(false);
    const [mousePosition, setMousePosition] = useState({});
    const [code, setCode] = useState();

    useEffect(() => {
        const text = ScenetoText(selectedFormat, scene, projectTitle);
        setCode(text);
    }, [scene, selectedFormat]);

    // Copies the text to the clipboard and displays a tooltip saying copied for two seconds
    const copyToClipboard = () => {
        navigator.clipboard.writeText(code).then(
            () => {
                setCopied(true);
                setTimeout(() => setCopied(false), 1000); // Hide tooltip after 2 seconds
            },
            (err) => {
                console.error("Could not copy text: ", err);
            }
        );
    };

    const handleClick = (e) => {
        setMousePosition({ x: e.clientX, y: e.clientY });
        copyToClipboard();
    };

    if (selectedFormat === "Parameters") {
        return null;
    }
    return (
        <div onClick={handleClick} className="code-container">
            <SyntaxHighlighter language={selectedFormat === "XACRO" ? "text" : "xml"} style={atomDark} customStyle={style}>
                {code}
            </SyntaxHighlighter>
            {copied && <Tooltip mousePosition={mousePosition}>Copied!</Tooltip>}
        </div>
    );
}
