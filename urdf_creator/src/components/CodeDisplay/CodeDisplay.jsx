import React, { useState } from 'react';
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import { ScenetoXML } from "../../utils/ScenetoXML";
import { ScenetoSDF } from "../../utils/ScenetoSDF";
import './CodeDisplay.css'; // Assuming you have a CSS file for this component

export default function CodeDisplay({ scene, projectTitle }) {
    const [format, setFormat] = useState('urdf');
    const [tooltip, setTooltip] = useState({ show: false, x: 0, y: 0 });

    const copyToClipboard = (text, e) => {
        navigator.clipboard.writeText(text).then(() => {
            const { clientX: x, clientY: y } = e; // Get mouse position
            setTooltip({ show: true, x, y });
            setTimeout(() => setTooltip({ show: false, x: 0, y: 0 }), 1000); // Hide tooltip after 2 seconds
        }, (err) => {
            console.error('Could not copy text: ', err);
        });
    };

    const renderContent = () => {
        const style = { fontSize: "12px", overflow: "auto", maxHeight: "50%", flexGrow: 1 };
        let code;

        switch (format) {
            case 'urdf':
                code = ScenetoXML(scene, projectTitle);
                break;
            case 'sdf':
                code = ScenetoSDF(scene, projectTitle);
                break;
            case 'xacro':
                code = "Xacro not currently supported. This format is for programmatically writing urdf files.";
                break;
            default:
                code = "";
        }

        return (
            <div onClick={(e) => copyToClipboard(code, e)} className="code-container">
                <SyntaxHighlighter language={format === 'xacro' ? 'text' : 'xml'} style={atomDark} customStyle={style}>
                    {code}
                </SyntaxHighlighter>
                {tooltip.show && (
                    <div className="tooltip" style={{ left: tooltip.x, top: tooltip.y }}>
                        Copied!
                    </div>
                )}
            </div>
        );
    };

    return (
        <div style={{ display: "flex", flexDirection: "column", maxHeight: "100%", flexGrow: 1, flexBasis: 0 }}>
            <div>
                <button className={format === 'urdf' ? 'button_selected' : ''} onClick={() => setFormat('urdf')}>URDF</button>
                <button className={format === 'sdf' ? 'button_selected' : ''} onClick={() => setFormat('sdf')}>SDF</button>
                <button className={format === 'xacro' ? 'button_selected' : ''} onClick={() => setFormat('xacro')}>XACRO</button>
            </div>
            {renderContent()}
        </div>
    );
}