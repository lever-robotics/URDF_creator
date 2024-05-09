import React, { useState, useContext } from 'react';
import { URDFContext } from '../URDFContext/URDFContext';

function InsertToolBar() {
    const { insertObject } = useContext(URDFContext);
    const [selectedShape, setSelectedShape] = useState("cube");

    const shapes = ["cube", "sphere", "cylinder"];

    return (
        <div style={{ display: 'flex', flexDirection: 'column', padding: 10 }}>
            {shapes.map(shape => (
                <button key={shape} onClick={() => {
                    setSelectedShape(shape);
                    insertObject(shape);
                }}>
                    {shape[0].toUpperCase() + shape.slice(1)}
                </button>
            ))}
        </div>
    );
}

export default InsertToolBar;
