import React from "react";

function ThreeDisplay({ mountRef }) {
    return (
        <div className="screen">
            <div className="display">
                <div ref={mountRef} style={{ width: "100%", height: "100%" }} />
            </div>
        </div>
    );
}

export default ThreeDisplay;
