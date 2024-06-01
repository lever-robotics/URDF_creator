import React from "react";

function ThreeDisplay({ mountRef }) {
    return (
        <div className="display">
            <div ref={mountRef} style={{ width: "100%", height: "100%" }} />
        </div>
    );
}

export default ThreeDisplay;
