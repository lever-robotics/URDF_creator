import React from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import { ScenetoXML } from "../../utils/ScenetoXML";

export default function CodeDisplay({ scene }) {
    return (
        <div style={{ display: "flex", flexDirection: "column", maxHeight: "100%", flexGrow: 1, flexBasis: 0 }}>
            <SyntaxHighlighter language="xml" style={atomDark} customStyle={{ fontSize: "12px", overflow: "auto", maxHeight: "50%", flexGrow: 1 }}>
                {ScenetoXML(scene)}
            </SyntaxHighlighter>
        </div>
    );
}
