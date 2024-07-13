import React, { useState } from 'react';
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import { ScenetoXML } from "../../utils/ScenetoXML";
import { ScenetoSDF } from "../../utils/ScenetoSDF";

export default function CodeDisplay({ scene }) {
    const [format, setFormat] = useState('urdf'); // Default format

    
    const renderContent = () => {
        const style = { fontSize: "12px", overflow: "auto", maxHeight: "50%", flexGrow: 1 };

        switch (format) {
            case 'urdf':
                return (
                    <SyntaxHighlighter language="xml" style={atomDark} customStyle={style}>
                        {ScenetoXML(scene)}
                    </SyntaxHighlighter>
                );
            case 'sdf':
                return (
                    <SyntaxHighlighter language="xml" style={atomDark}customStyle={style}>
                        {ScenetoSDF(scene)}
                    </SyntaxHighlighter>
                );
            case 'xacro':
                return (
                    <SyntaxHighlighter language="text" style={atomDark} customStyle={style}>
                        Xacro not currently supported. This format is for programmatically writing urdf files.
                    </SyntaxHighlighter>
                );
            default:
                return null;
        }
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
