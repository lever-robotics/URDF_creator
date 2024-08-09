import React from "react";


function InsertTool({ addObject }) {
    return (
        <div style={{ marginTop: "10px" }} className="column-box">
            Add Link
            <button onClick={() => addObject("cube")}>Add Cube</button>
            <button onClick={() => addObject("sphere")}>Add Sphere</button>
            <button onClick={() => addObject("cylinder")}>Add Cylinder</button>
        </div>
    );
}

export default InsertTool;
