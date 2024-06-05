import React from "react";
import { handleDownload } from "../../../utils/HandleDownload";

const DownloadRobotPackage = (scene, projectTitle) => {

    // Kept incase we want to place under code editor. Untested

    return (
        <div>
            <button 
                onClick={() => {
                    handleDownload(scene, 'urdf', projectTitle)
                }}>
                Download Robot Description Package
            </button>
        </div>
    );
};

export default DownloadRobotPackage;
