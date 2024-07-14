import React, { useState, useEffect } from 'react';
import { openDB } from 'idb';

function MeshParameters({ selectedObject, setMesh }) {
    const [mesh, setMeshTemp] = useState("");
    const [files, setFiles] = useState([]);

    useEffect(() => {
        if (selectedObject) {
            setMeshTemp(selectedObject.userData.stlfile || "");
        }
    }, [selectedObject]);

    const handleMeshChange = (e) => {
        setMeshTemp(e.target.value);
        setMesh(selectedObject, e.target.value);
    };

    useEffect(() => {
        const getFiles = async () => {
            const db = await openDB('stlFilesDB', 1);
            const files = await db.getAll('files');
            setFiles(files);
        };
        getFiles();
    }, []);

    return (
        <div>
            <strong>Mesh (only for visual):</strong>
            <select value={mesh} onChange={handleMeshChange}>
                <option value="">No Mesh</option>
                {files.map(file => (
                    <option key={file.name} value={file.name}>{file.name}</option>
                ))}
            </select>
        </div>
    );
}

export default MeshParameters;