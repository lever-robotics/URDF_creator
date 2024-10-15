import { openDB } from "idb";
import type React from "react";
import { useEffect, useState } from "react";
import type { Collision, Visual } from "../../../../Models/VisualCollision";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import styles from "./Parameter.module.css";
import Property from "./Property";
import Section from "./Section";

type MeshParametersProps = {
    selectedObject: Visual | Collision;
    threeScene: ThreeScene;
};

function MeshParameters({ selectedObject, threeScene }: MeshParametersProps) {
    const [files, setFiles] = useState<File[]>([]);
    const [geometryValue, setGeometryValue] = useState(
        selectedObject.shape === "mesh"
            ? selectedObject.stlfile
            : selectedObject.shape,
    );

    const handleGeometryChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        //check if value is cube sphree or cylinder then set geometry as that with blank file name
        if (
            e.target.value === "cube" ||
            e.target.value === "sphere" ||
            e.target.value === "cylinder"
        ) {
            selectedObject.setGeometry(e.target.value, "");
            setGeometryValue(e.target.value);
        } else {
            selectedObject.setGeometry("mesh", e.target.value);
            setGeometryValue("mesh");
        }
        threeScene.dispatchEvent("parameters");
        threeScene.forceUpdateCode();
    };

    const loadFiles = async () => {
        try {
            // Open the database
            const db = await openDB("stlFilesDB", 1, {
                upgrade(db) {
                    if (!db.objectStoreNames.contains("files")) {
                        db.createObjectStore("files", { keyPath: "name" });
                    }
                },
            });

            // Get all files from the 'files' store
            const files = await db.getAll("files");

            // Set the files if they exist, or return an empty array
            setFiles(files || []);
        } catch (error) {
            console.error("Error retrieving files:", error);
            setFiles([]); // Set an empty array on error to avoid breaking the UI
        }
    };

    // useEffect(() => {
    //     // Load files when the component is first rendered
    //     loadFiles();
    // }, []);

    return (
        <Property name="Choose Geometry">
            (computationally intensive if mesh applied as collision geometry):
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
        </Property>
    );
}

export default MeshParameters;
