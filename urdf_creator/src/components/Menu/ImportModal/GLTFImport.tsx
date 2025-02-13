import type React from "react";
import "./Import.css";
import { useRef } from "react";
import type { Object3D } from "three";
import { handleUpload } from "../../../utils/HandleUpload";

const GLTFImport = ({
    onClose,
    loadScene,
}: {
    onClose: () => void;
    loadScene: (object: Object3D) => void;
}) => {
    const inputGTLFFile = useRef<HTMLInputElement>(null);

    /* Annoying File Upload Logic
      1. Clicking Upload File activates onFileUpload() which 'clicks' the input element
      2. The input element has an onChange listener that uploads the file using the handleFileChange() function which calls handleUpload() 
    */
    const onFileUpload = () => {
        if (inputGTLFFile.current) inputGTLFFile.current.click();
    };
    const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
        if (!e.target.files) return;
        const file = e.target.files[0];
        const type = file.name.split(".").pop();
        if (!type) return;
        const group = await handleUpload(file, type);
        const base_link = group.children[0];
        loadScene(base_link);
        onClose();
    };

    const onClick = () => {
        onFileUpload();
        // onClose();
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="import-title">
                    <h3>Import a GLTF file</h3>
                </div>
                <div className="import-box">
                    <button
                        className="import-button"
                        onClick={onClick}
                        type="button"
                    >
                        Import GLTF
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>
                    A GLTF file is a local file that you would have exported
                    from this URDF creator to save a local copy of your project.
                    It takes all the information from your defined robot and
                    condenses it into a small file that can be imported here.
                </p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img
                        src={`${import.meta.env.BASE_URL}/statics/gltf_share.png`}
                        alt="gltf Share"
                        className="gltfgraphic"
                    />
                </div>
                <div className="image-description">
                    Share your robot description in this software with developer
                    teams to modify the same robot description
                </div>
            </div>
            <div className="description-container">
                <p>
                    For understanding the capabilities of GLTF files in
                    versioning your defined robot, refer to the{" "}
                    <a href="https://roboeverything.com">documentation</a>.
                </p>
            </div>
            <input
                type="file"
                ref={inputGTLFFile}
                style={{ display: "none" }}
                onChange={handleFileChange}
                accept=".gltf"
            />
        </div>
    );
};

export default GLTFImport;
