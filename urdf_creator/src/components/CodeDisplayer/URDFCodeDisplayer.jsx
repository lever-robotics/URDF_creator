import React, { useContext, useState, useEffect } from 'react';
import { URDFCodeContext } from '../URDFContext/URDFCodeContext';
import DownloadRobotPackage from '../DownloadRobotPackage/DownloadRobotPackage';

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
        <div style={{ margin: '10px', display: 'flex', flexDirection: 'column'}}>
            <textarea
                value={editCode}
                onChange={handleCodeChange}
                style={{
                    flexGrow: 1,
                    width: '100%',
                    fontFamily: 'monospace',
                    fontSize: '14px',
                    border: '1px solid #ccc',
                    outline: 'none',
                    padding: '10px',
                    overflowY: 'auto',
                    borderRadius: '4px',
                    boxSizing: 'border-box'
                }}
            />
            <div className='row-spaced' style={{ padding: '6px 12px', marginTop: '10px' }}>
                <button onClick={handleSave} >Save</button>
                <DownloadRobotPackage />
            </div>
            
        </div>
    );
};

export default URDFCodeDisplayer;
