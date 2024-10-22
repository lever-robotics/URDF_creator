import "./Export.css";
import Frame, { type Frameish } from "../../../Models/Frame";
import {
    handleDownload,
    handleProjectDownload,
} from "../../../utils/HandleDownload";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import { compressScene } from "../../ThreeDisplay/TreeUtils";

type Props = {
    onClose: () => void;
    rootFrame: Frameish;
    projectTitle: string;
    threeScene: ThreeScene;
};

const ExportGLTF: React.FC<Props> = ({
    onClose,
    rootFrame,
    projectTitle,
    threeScene,
}) => {
    const handleGLTFExport = () => {
        if (!rootFrame) return;
        const compressedScene = compressScene(rootFrame);
        handleProjectDownload(compressedScene, projectTitle);
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="export-title">
                    <h3>Export a GLTF file</h3>
                </div>
                <div className="export-box">
                    <button
                        className="export-button"
                        onClick={() => {
                            handleGLTFExport();
                            onClose();
                        }}
                        type="button"
                    >
                        Export a GLTF file
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>
                    A GLTF file is a local file that you would have exported
                    from this URDF creator to save a local copy of your project.
                    It takes all the information from your defined robot and
                    condenses it into a small file that can be import back into
                    this tool from anywhere.
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
        </div>
    );
};

export default ExportGLTF;
