import React, { useContext, useState, useEffect } from 'react';
import { URDFCodeContext } from '../URDFContext/URDFCodeContext';

const URDFCodeDisplayer = () => {
    const { currentURDFCode, updateURDFCode } = useContext(URDFCodeContext);
    const [editCode, setEditCode] = useState('');

    useEffect(() => {
        setEditCode(currentURDFCode);
    }, [currentURDFCode]);

    const handleCodeChange = (event) => {
        setEditCode(event.target.value);
    };

    const handleSave = () => {
        updateURDFCode(editCode);
    };

    return (
        <div style={{ margin: '10px' }}>
            <textarea
                value={editCode}
                onChange={handleCodeChange}
                style={{
                    width: '100%',
                    minHeight: '400px',
                    fontFamily: 'monospace',
                    fontSize: '14px',
                    border: '1px solid #ccc',
                    outline: 'none',
                    padding: '10px',
                    overflowY: 'auto',
                    borderRadius: '4px',
                }}
            />
            <button onClick={handleSave} style={{ padding: '6px 12px', marginTop: '10px' }}>Save</button>
        </div>
    );
};

export default URDFCodeDisplayer;
