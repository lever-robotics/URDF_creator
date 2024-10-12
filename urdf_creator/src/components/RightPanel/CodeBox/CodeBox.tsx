import { useEffect, useState } from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import ScenetoText from "../../../utils/ScenetoText";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import styles from "./CodeBox.module.css";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 * @param {string} selectedFormat
 */

type Props = {
    projectTitle: string;
    threeScene: ThreeScene;
    selectedFormat: string;
    updateCode: number;
};

export default function CodeBox({
    projectTitle,
    threeScene,
    selectedFormat,
    // updateCode,
}: Props) {
    const style = {
        fontSize: "12px",
        overflow: "auto",
        flexGrow: 1,
        backgroundColor: "#14252E",
        borderRadius: "0px",
        margin: "0",
        padding: "4px 8px",
    };
    const [copied, setCopied] = useState(false);
    const [code, setCode] = useState("");
    const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });

    // Writing text to the screen can be costly and a huge perfomance hit. This useEffect ensures that the urdf text is only updated on it's dependencies and when the ThreeScene calls for a code update
    useEffect(() => {
        const updateCode = () => {
            const text = ScenetoText(selectedFormat, threeScene, projectTitle);
            setCode(text);
        };

        updateCode();
        threeScene.addEventListener("updateCode", updateCode);

        return () => {
            threeScene.removeEventListener("updateCode", updateCode);
        };
    }, [selectedFormat, threeScene, projectTitle]);

    // Copies the text to the clipboard and displays a tooltip saying copied for two seconds
    const copyToClipboard = () => {
        navigator.clipboard.writeText(code).then(
            () => {
                setCopied(true);
                setTimeout(() => setCopied(false), 1000); // Hide tooltip after 2 seconds
            },
            (err) => {
                console.error("Could not copy text: ", err);
            },
        );
    };

    const handleClick = (e: React.MouseEvent) => {
        setMousePosition({ x: e.clientX, y: e.clientY });
        copyToClipboard();
    };

    if (selectedFormat === "Parameters") return null;

    return (
        <div onClick={handleClick} className={styles.codeContainer}>
            <SyntaxHighlighter
                language={selectedFormat === "XACRO" ? "text" : "xml"}
                style={atomDark}
                customStyle={style}
            >
                {code}
            </SyntaxHighlighter>
            {copied && <Tooltip mousePosition={mousePosition} />}
        </div>
    );
}

/**
 * @param {*} mousePosition
 * @returns JSX component that displays the text 'Copied!'
 */
function Tooltip({
    mousePosition,
}: {
    mousePosition: { x: number; y: number };
}) {
    return (
        <div
            className={styles.tooltip}
            style={{ left: mousePosition.x, top: mousePosition.y }}
        >
            Copied!
        </div>
    );
}
