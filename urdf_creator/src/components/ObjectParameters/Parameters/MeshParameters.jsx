import React, { useState, useEffect } from "react";
import ToggleSection from "../ToggleSection";
import { openDB } from "idb";

function MeshParameters({ selectedObject, stateFunctions }) {
    const [files, setFiles] = useState([]);

    const handleMeshChange = (e) => {
        stateFunctions.setMesh(selectedObject, e.target.value);
    };

    const loadFiles = async () => {
        try {
            // Open the database
            const db = await openDB("stlFilesDB", 1, {
                upgrade(db) {
                    if (!db.objectStoreNames.contains("files")) {
                        db.createObjectStore("files", { keyPath: "name" });
                    }
                }
            });

            // Get all files from the 'files' store
            const files = await db.getAll("files");

            // Set the files if they exist, or return an empty array
            setFiles(files || []);
        } catch (error) {
            console.error('Error retrieving files:', error);
            setFiles([]); // Set an empty array on error to avoid breaking the UI
        }
    };

    useEffect(() => {
        // Load files when the component is first rendered
        loadFiles();
    }, []);

    return (
        <ToggleSection title="Mesh Parameters" open={false}>
            <strong>Mesh (only for visual):</strong>
            <select
                value={selectedObject.userData.stlfile}
                onChange={handleMeshChange}
                onClick={loadFiles} // Load files when the selection bar is clicked
            >
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
