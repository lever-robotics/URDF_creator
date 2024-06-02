import React from "react";

const Toolbar = ({ setTransformMode }) => {
    return (
        <div style={{ marginTop: "10px", height: "40px", pointerEvents: "auto" }} className="row-space-between">
            <div className="row-spaced">
                <button onClick={() => setTransformMode("translate")}>Translate</button>
                <button onClick={() => setTransformMode("rotate")}>Rotate</button>
                <button onClick={() => setTransformMode("scale")}>Scale</button>
            </div>
        </div>
    );
};

export default Toolbar;
