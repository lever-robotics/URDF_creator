import { useState, useEffect } from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import ScenetoText from "../../../utils/ScenetoText";
import "./CodeBox.css";
import Frame from "../../../Models/Frame";
import { StateFunctionsType } from "../../SceneState";
import {Scene} from "three";
import ThreeScene from "../../ThreeDisplay/ThreeSceneObject";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 * @param {string} selectedFormat
 */

type Props = {
    projectTitle: string,
    threeScene: ThreeScene,
    selectedFormat: string,
    updateCode: number
}

export default function CodeBox({ projectTitle, threeScene, selectedFormat, updateCode }: Props) {
    const style = {
        fontSize: "12px",
        overflow: "auto",
        height: "100%",
        width: "100%",
        flexGrow: 1,

    };
    const [copied, setCopied] = useState(false);
    const [mousePosition, setMousePosition] = useState({x: 0, y:0});
    const [code, setCode] = useState<string>();

    useEffect(() => {
        if (selectedFormat !== "Parameters") {
            const text = ScenetoText(selectedFormat, threeScene.scene, projectTitle);
            setCode(text);
        }
    }, [updateCode, selectedFormat, projectTitle]);

    // Copies the text to the clipboard and displays a tooltip saying copied for two seconds
    const copyToClipboard = () => {
        navigator.clipboard.writeText(code!).then(
            () => {
                setCopied(true);
                setTimeout(() => setCopied(false), 1000); // Hide tooltip after 2 seconds
            },
            (err) => {
                console.error("Could not copy text: ", err);
            }
        );
    };

    const handleClick = (e: React.MouseEvent) => {
        setMousePosition({ x: e.clientX, y: e.clientY });
        copyToClipboard();
    };

    if (selectedFormat === "Parameters") {
        return null;
    }
    return (
        <div onClick={handleClick} className="code-container">
            <SyntaxHighlighter language={selectedFormat === "XACRO" ? "text" : "xml"} style={atomDark} customStyle={style}>
                {code!}
            </SyntaxHighlighter>
            {copied && <Tooltip mousePosition={mousePosition} />}
        </div>
    );
}

/**
 * @param {*} mousePosition
 * @returns JSX component that displays the text 'Copied!'
 */
function Tooltip({ mousePosition }: {mousePosition: {x: number, y: number}}) {
    return (
        <div className="tooltip" style={{ left: mousePosition.x, top: mousePosition.y }}>
            Copied!
        </div>
    );
}
