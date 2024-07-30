import React, { useState, useEffect } from "react";
import ToggleSection from "../ToggleSection";
import { openDB } from "idb";

function MeshParameters({ selectedObject, stateFunctions }) {
    const [files, setFiles] = useState([]);

    const handleMeshChange = (e) => {
        stateFunctions.setMesh(selectedObject, e.target.value);
    };

    useEffect(() => {
        const getFiles = async () => {
            const db = await openDB("stlFilesDB", 1);
            const files = await db.getAll("files");
            setFiles(files);
        };
        try{
            // If I don't have the DB created then this crashes the program. Can we build some error checking into this so even if no STL files are uploaded the program works?
            
            // getFiles();
        }
        catch (error) {
            console.error(error);
        }
    }, []);

    return (
        <ToggleSection title="Mesh Parameters">
            <strong>Mesh (only for visual):</strong>
            <select value={selectedObject.mesh} onChange={handleMeshChange}>
                <option value="">No Mesh</option>
                {files.map((file) => (
                    <option key={file.name} value={file.name}>
                        {file.name}
                    </option>
                ))}
            </select>
        </ToggleSection>
    );
}

export default MeshParameters;
