import type React from "react";
import "./Import.css";
import { openDB } from "idb";
import { useRef } from "react";

const STLImport = ({ onClose }: { onClose: () => void }) => {
    const inputSTLFile = useRef<HTMLInputElement>(null);

    // Function to open the file selector
    const onSTLFileUpload = () => {
        if (inputSTLFile.current) inputSTLFile.current.click();
    };

    // Function to handle the selected file
    const handleSTLFileChange = async (
        event: React.ChangeEvent<HTMLInputElement>,
    ) => {
        const selectedFile = event.target.files?.[0];
        if (selectedFile) {
            try {
                const db = await openDB("stlFilesDB", 1, {
                    upgrade(db) {
                        if (!db.objectStoreNames.contains("files")) {
                            db.createObjectStore("files", { keyPath: "name" });
                        }
                    },
                });

                await db.put("files", {
                    name: selectedFile.name,
                    file: selectedFile,
                });

                // Clear the input value after upload
                event.target.value = "";
                onClose(); //close the modale after complete
            } catch (error) {
                console.error("Error during file upload:", error);
            }
        } else {
            console.log("No file selected.");
        }
    };

    // Function to handle button click (file input + close action)
    const onClick = () => {
        onSTLFileUpload();
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="import-title">
                    <h3>Import an STL File</h3>
                </div>
                <div className="import-box">
                    <button
                        className="import-button"
                        onClick={onClick}
                        type="button"
                    >
                        Import STL
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>
                    Am STL (Stereolithography) files represent the triangulated
                    surface geometry of a 3D model using a mesh of triangles. In
                    a URDF, STLs are typically used for the visual
                    representation of links due to their detailed appearance, as
                    they can be computationally expensive for simulations. While
                    mainly used for visuals, STLs can also define collision
                    properties if needed. Once imported, you can assign your STL
                    as the visual geometry or collision geometry for any link in
                    your URDF in the link parameters menu.
                </p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img
                        src={"/statics/STL_Description.png"}
                        alt="STL Mesh"
                        className="stlgraphic"
                    />
                </div>
                <div className="image-description">
                    STLs can be exported from most CAD Modeling software
                    including but not limited to Solidworks, Blender, AutoCad,
                    Onshape and several others. A public repository of them can
                    also be found online, one place being{" "}
                    <a href="https://www.thingiverse.com/">Thingiverse</a>
                </div>
            </div>
            <div className="description-container">
                <p>
                    For converting 3D models from CAD software to STL or DAE
                    formats, refer to the{" "}
                    <a href="https://roboeverything.com">documentation</a>.
                </p>
            </div>
            <input
                type="file"
                ref={inputSTLFile}
                style={{ display: "none" }}
                onChange={handleSTLFileChange}
                accept=".stl"
            />
        </div>
    );
};

export default STLImport;
