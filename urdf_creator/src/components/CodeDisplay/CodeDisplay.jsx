import React from "react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { atomDark } from "react-syntax-highlighter/dist/esm/styles/prism";
import { ScenetoXML } from "../../utils/ScenetoXML";

export default function CodeDisplay({ scene }) {

    return (
        <div
            style={{ margin: "10px", display: "flex", flexDirection: "column", overflowX: "scroll" }}
        >
            <SyntaxHighlighter
                language="xml"
                style={atomDark}
                customStyle={{ fontSize: "12px" }} // Inline style for font size
            >
                {ScenetoXML(scene)}
            </SyntaxHighlighter>
        </div>
    );
};
