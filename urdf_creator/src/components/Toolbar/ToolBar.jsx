import React from "react";

const Toolbar = ({ setTransformMode, selectedObject }) => {
    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <button onClick={() => setTransformMode("translate", selectedObject)}>Translate</button>
                <button onClick={() => setTransformMode("rotate", selectedObject)}>Rotate</button>
                <button onClick={() => setTransformMode("scale", selectedObject)}>Scale</button>
            </div>
        </div>
    );
};

export default Toolbar;
