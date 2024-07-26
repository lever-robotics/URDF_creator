import React, { useState, useEffect } from "react";

function ScaleParameters({ selectedObject, transformObject }) {
    const [scaleX, setScaleX] = useState("");
    const [scaleY, setScaleY] = useState("");
    const [scaleZ, setScaleZ] = useState("");

    useEffect(() => {
        if (selectedObject) {
            setScaleX(selectedObject.scale.x.toFixed(2));
            setScaleY(selectedObject.scale.y.toFixed(2));
            setScaleZ(selectedObject.scale.z.toFixed(2));
        }
    }, [JSON.stringify(selectedObject.scale)]);

    const handleChange = (axis, value) => {
        const newValue = parseFloat(value);
        if (isNaN(newValue)) return;
        const x = parseFloat(scaleX);
        const y = parseFloat(scaleY);
        const z = parseFloat(scaleZ);

        if (selectedObject.shape === "sphere") {
            transformObject(selectedObject, "scale", x, x, x);
        } else if (selectedObject.shape === "cylinder") {
            if (axis === "x" || axis === "y") {
                transformObject(selectedObject, "scale", x, x, z);
            } else {
                transformObject(selectedObject, "scale", x, y, z);
            }
        } else {
            transformObject(selectedObject, "scale", x, y, z);
        }
    };

    const handleBlur = (axis, value) => {
        handleChange(axis, value);
    };

    const handleKeyDown = (e, axis, value) => {
        if (e.key === "Enter") {
            handleBlur(axis, value);
        }
    };

    let scaleInputs;
    switch (selectedObject.shape) {
        case "sphere":
            scaleInputs = (
                <li>
                    Radius:
                    <input
                        type="number"
                        value={scaleX / 2}
                        onChange={(e) => setScaleX(e.target.value * 2)}
                        onBlur={() => handleBlur("x", scaleX * 2)}
                        onKeyDown={(e) => handleKeyDown(e, "x", scaleX * 2)}
                    />
                    <span className="units">m</span>
                </li>
            );
            break;
        case "cylinder":
            scaleInputs = (
                <>
                    <li>
                        Radius:
                        <input
                            type="number"
                            value={scaleX / 2}
                            onChange={(e) => setScaleX(2 * e.target.value)}
                            onBlur={() => handleBlur("x", 2 * scaleX)}
                            onKeyDown={(e) => handleKeyDown(e, "x", 2 * scaleX)}
                        />
                        <span className="units">m</span>
                    </li>
                    <li>
                        Height:
                        <input
                            type="number"
                            value={scaleZ}
                            onChange={(e) => setScaleZ(e.target.value)}
                            onBlur={() => handleBlur("z", scaleZ)}
                            onKeyDown={(e) => handleKeyDown(e, "z", scaleZ)}
                        />
                        <span className="units">m</span>
                    </li>
                </>
            );
            break;
        default:
            scaleInputs = (
                <>
                    <li>
                        X:
                        <input
                            type="number"
                            value={scaleX}
                            onChange={(e) => setScaleX(e.target.value)}
                            onBlur={() => handleBlur("x", scaleX)}
                            onKeyDown={(e) => handleKeyDown(e, "x", scaleX)}
                        />
                        <span className="units">m</span>
                    </li>
                    <li>
                        Y:
                        <input
                            type="number"
                            value={scaleY}
                            onChange={(e) => setScaleY(e.target.value)}
                            onBlur={() => handleBlur("y", scaleY)}
                            onKeyDown={(e) => handleKeyDown(e, "y", scaleY)}
                        />
                        <span className="units">m</span>
                    </li>
                    <li>
                        Z:
                        <input
                            type="number"
                            value={scaleZ}
                            onChange={(e) => setScaleZ(e.target.value)}
                            onBlur={() => handleBlur("z", scaleZ)}
                            onKeyDown={(e) => handleKeyDown(e, "z", scaleZ)}
                        />
                        <span className="units">m</span>
                    </li>
                </>
            );
            break;
    }

    return <ul>{scaleInputs}</ul>;
}

export default ScaleParameters;
