import React, { useState } from "react";

const Toolbar = ({ setTransformMode, selectedObject }) => {
    // For keeping track of which tool is selected to highlight that button for the user
    const [tool, setTool] = useState("translate");

    return (
        <div
            style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }}
            className="row-space-between">
            <div className="row-spaced">
                <button
                    className={tool === "translate" ? "button_selected" : ""}
                    onClick={() => {
                        setTransformMode("translate", selectedObject);
                        setTool("translate");
                    }}>
                    Translate
                </button>
                <button
                    className={tool === "rotate" ? "button_selected" : ""}
                    onClick={() => {
                        setTransformMode("rotate", selectedObject);
                        setTool("rotate");
                    }}>
                    Rotate
                </button>
                <button
                    className={tool === "scale" ? "button_selected" : ""}
                    onClick={() => {
                        setTransformMode("scale", selectedObject);
                        setTool("scale");
                    }}>
                    Scale
                </button>
            </div>
        </div>
    );
};

export default Toolbar;
