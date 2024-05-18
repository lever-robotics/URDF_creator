import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useStateContext } from '../URDFContext/StateContext';
import DownloadRobotPackage from '../DownloadRobotPackage/DownloadRobotPackage';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { atomDark } from 'react-syntax-highlighter/dist/esm/styles/prism';
import { isValidURDF } from '../../utils/URDFValidator'; // Assuming you have this function defined
import debounce from 'lodash.debounce'; // You can install lodash.debounce or implement your own debouncer

const URDFCodeDisplayer = () => {
    const { state, dispatch } = useStateContext();
    const [editCode, setEditCode] = useState('');
    const [isEditing, setIsEditing] = useState(false);
    const textAreaRef = useRef(null);

    useEffect(() => {
        setEditCode(state.URDFCode);
    }, [state.URDFCode]);

    const handleCodeChange = (event) => {
        setEditCode(event.target.value);
        debouncedValidateAndDispatch(event.target.value);
    };

    const handleDoubleClick = () => {
        setIsEditing(true);
        setTimeout(() => {
            textAreaRef.current && textAreaRef.current.focus();
        }, 0);
    };

    const debouncedValidateAndDispatch = useCallback(debounce((code) => {
        console.log('Validating URDF code...')
        const isValid = isValidURDF(code);
        console.log('URDF code is valid:', isValid);
        if (isValid) {
            dispatch({ type: 'SET_URDF_CODE', payload: code });
            console.log(state.URDFCode);
        }
    }, 300), []);

    const handleBlur = () => {
        setIsEditing(false);
    };

    return (
        <div style={{ margin: '10px', display: 'flex', flexDirection: 'column' }}>
            {isEditing ? (
                <textarea
                    ref={textAreaRef}
                    value={editCode}
                    onChange={handleCodeChange}
                    onBlur={handleBlur}
                    style={{
                        flexGrow: 1,
                        width: '100%',
                        fontFamily: 'monospace',
                        fontSize: '12px', // Smaller text size for the textarea
                        border: '1px solid #ccc',
                        outline: 'none',
                        padding: '10px',
                        overflowY: 'auto',
                        borderRadius: '4px',
                        boxSizing: 'border-box'
                    }}
                />
            ) : (
                <div
                    onDoubleClick={handleDoubleClick}
                    style={{
                        flexGrow: 1,
                        width: '100%',
                        padding: '10px',
                        overflowY: 'auto',
                        borderRadius: '4px',
                        boxSizing: 'border-box',
                        backgroundColor: '#2d2d2d',
                        color: '#f8f8f2',
                        cursor: 'pointer'
                    }}
                >
                    <SyntaxHighlighter
                        language="xml"
                        style={atomDark}
                        customStyle={{ fontSize: '12px' }} // Inline style for font size
                    >
                        {editCode}
                    </SyntaxHighlighter>
                </div>
            )}
            <div className='row-spaced' style={{ padding: '6px 12px', marginTop: '10px' }}>
                <DownloadRobotPackage />
            </div>
        </div>
    );
};

export default URDFCodeDisplayer;
