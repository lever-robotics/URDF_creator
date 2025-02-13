import type React from "react";
import "./Export.css";
import { handleDownload } from "../../../utils/HandleDownload";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";

type Props = {
    onClose: () => void;
    scene: ThreeScene;
    projectTitle: string;
};

const ExportURDFPackage: React.FC<Props> = ({
    onClose,
    scene,
    projectTitle,
}) => {
    const handleURDFExport = () => {
        handleDownload(scene, "urdf", projectTitle);
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="export-title">
                    <h3>Export URDF file</h3>
                </div>
                <div className="export-box">
                    <button
                        className="export-button"
                        onClick={() => {
                            handleURDFExport();
                            onClose();
                        }}
                        type="button"
                    >
                        Export URDF file
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>
                    This downloads the single URDF file to be used with a
                    compatible simulator or software.
                </p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img
                        src={`${import.meta.env.BASE_URL}/statics/urdf_folder.png`}
                        alt="URDF file"
                        className="urdfgraphic"
                    />
                </div>
                <div className="image-description">
                    Download the URDF file to be used for uploading on several
                    platforms including Unity, Issac Sim, and ROS2.
                </div>
            </div>
            <div className="description-container">
                <p>
                    To understand more on how to use the URDF file and its use,
                    refer to the{" "}
                    <a href="https://roboeverything.com">documentation</a>.
                </p>
            </div>
        </div>
    );
};

export default ExportURDFPackage;
