import React, { useContext, useState, useEffect } from 'react';
import { URDFCodeContext } from '../URDFContext/URDFCodeContext';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { vscDarkPlus } from 'react-syntax-highlighter/dist/esm/styles/prism';
import { markup } from 'react-syntax-highlighter/dist/esm/languages/prism';

const URDFCodeDisplayer = () => {
    const { currentURDFCode, updateURDFCode } = useContext(URDFCodeContext);
    const [editCode, setEditCode] = useState(currentURDFCode);

    useEffect(() => {
        setEditCode(currentURDFCode);
    }, [currentURDFCode]);

    const handleCodeChange = (event) => {
        setEditCode(event.target.value);
    };

    const handleSave = () => {
        // Update the context with the current editing text
        updateURDFCode(editCode);
    };

    return (
        <div style={{ margin: '10px' }}>
            <div
                contentEditable
                spellCheck="false"
                onInput={handleCodeChange}
                onBlur={handleCodeChange}
                style={{
                    width: '100%',
                    minHeight: '400px',
                    fontFamily: 'monospace',
                    fontSize: '14px',
                    border: '1px solid #ccc',
                    outline: 'none',
                    padding: '10px',
                    overflowY: 'auto',
                    backgroundColor: '#f5f5f5',
                    borderRadius: '4px',
                    whiteSpace: 'pre-wrap',
                    position: 'relative',
                }}
            >
                <SyntaxHighlighter language="markup" style={vscDarkPlus} customStyle={{ background: 'none', padding: 0, margin: 0 }}>
                    {editCode}
                </SyntaxHighlighter>
            </div>
            <button onClick={handleSave} style={{ padding: '6px 12px', marginTop: '10px' }}>Save</button>
        </div>
    );
};

export default URDFCodeDisplayer;
