import React, { useState, useEffect } from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import ScenetoText from "../../utils/ScenetoText";
import "./CodeDisplay.css"; // Assuming you have a CSS file for this component

/**
 * Displays buttons to determine with format the text representation of the scene should be displayed in as well as a text box containing the text
 * @param {Scene} scene The scene object
 * @param {string} projectTitle  
 * @returns Format buttons and a text box
 */
export default function CodeDisplay({ scene, projectTitle }) {
    const [selectedFormat, setSelectedFormat] = useState("URDF");

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
            <div>
                <button
                    className={
                        selectedFormat === "URDF" ? "button_selected" : ""
                    }
                    onClick={handleClick}>
                    URDF
                </button>
                <button
                    className={
                        selectedFormat === "SDF" ? "button_selected" : ""
                    }
                    onClick={handleClick}>
                    SDF
                </button>
                <button
                    className={
                        selectedFormat === "XACRO" ? "button_selected" : ""
                    }
                    onClick={handleClick}>
                    XACRO
                </button>
            </div>
            <CodeBox
                scene={scene}
                projectTitle={projectTitle}
                selectedFormat={selectedFormat}
            />
        </div>
    );
}

/**
 * Contains the text representation of the scene object
 * @param {Scene} scene The scene object
 * @param {string} projectTitle
 * @param {string} selectedFormat The format the text representation of the scene will be displayed in
 * @returns A text box with syntax highlighting and text representation of the scene
 */
function CodeBox({ scene, projectTitle, selectedFormat }) {
    const style = {
        fontSize: "12px",
        overflow: "auto",
        maxHeight: "50%",
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

    return (
        <div onClick={handleClick} className="code-container">
            <SyntaxHighlighter
                language={selectedFormat === "XACRO" ? "text" : "xml"}
                style={atomDark}
                customStyle={style}>
                {code}
            </SyntaxHighlighter>
            {copied && <Tooltip mousePosition={mousePosition} />}
        </div>
    );
}

/**
 * A tooltip meant to display below the mouse when the mouse clicks on the codebox
 * @param {*} mousePosition The x and y coords for the mouse
 * @returns JSX component that displays the text 'Copied!'
 */
function Tooltip({ mousePosition }) {
    return (
        <div
            className="tooltip"
            style={{ left: mousePosition.x, top: mousePosition.y }}>
            Copied!
        </div>
    );
}
