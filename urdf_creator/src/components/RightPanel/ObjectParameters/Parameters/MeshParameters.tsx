import React, { useState, useEffect } from "react";
import Section from "../Section";
import { openDB } from "idb";
import ItemParameterProps from "../ItemParameterProps";
import { Frameish } from "../../../../Models/Frame";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import ThreeScene from "../../../ThreeDisplay/ThreeScene";
import styles from "../ObjectParameters.module.css";

type MeshParametersProps = {
    selectedObject?: Frameish,
    selectedItem?: Visual | Collision,
    threeScene: ThreeScene,
}

function MeshParameters({ selectedObject, selectedItem, threeScene }: MeshParametersProps) {
    if (!selectedObject) return;
    const [files, setFiles] = useState<any[]>([]);
    const [geometryValue, setGeometryValue] = useState(
        selectedItem!.shape === "mesh" ? selectedItem!.stlfile : selectedItem!.shape
    );

    useEffect(() => {
        setGeometryValue(selectedItem!.shape === "mesh" ? selectedItem!.stlfile : selectedItem!.shape);
    }
    , [JSON.stringify(selectedItem!.shape)]);

    const handleGeometryChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        //check if value is cube sphree or cylinder then set geometry as that with blank file name
        if (e.target.value === "cube" || e.target.value === "sphere" || e.target.value === "cylinder") {
            selectedItem!.setGeometry(e.target.value, "");
            threeScene.forceUpdateBoth();
            return;
        } else {
            // Set the geometry type to mes
            selectedItem!.setGeometry("mesh", e.target.value);
        }
        threeScene.forceUpdateBoth();
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
        <Section title="">
            Choose Geometry (computationally intensive if mesh applied as collision geometry):
            <select
                value={geometryValue}
                onChange={handleGeometryChange}
                onClick={loadFiles} // Load files when the selection bar is clicked
                className={styles.select}
            >
                <option value="cube">Cube</option>
                <option value="sphere">Sphere</option>
                <option value="cylinder">Cylinder</option>
                {files.map((file) => (
                    <option key={file.name} value={file.name}>
                        {file.name}
                    </option>
                ))}
            </select>
        </Section>
    );
}

export default MeshParameters;
