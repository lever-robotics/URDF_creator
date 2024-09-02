import './Export.css';
import urdfObjectManager from '../../../Models/urdfObjectManager';
import { handleDownload } from '../../../utils/HandleDownload';

const ExportGLTF = ({ onClose, getBaseLink, projectTitle }) => {

    const handleGLTFExport = () => {
        const manager = new urdfObjectManager();
        const compressedScene = manager.compressScene(getBaseLink());
        handleDownload(compressedScene, "gltf", projectTitle);
    };

    return (
        <div className='des-container'>
            <div className="header">
                <div className="export-title">
                    <h3>export a GLTF file</h3>
                </div>
                <div className="export-box">
                    <button className='export-button' onClick={() => {handleGLTFExport(); onClose();}}>Export ROS2 Package</button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>A GLTF file is a local file that you would have exported from this URDF creator to save a local copy of your project. It takes all the information from your defined robot and condenses it into a small file that can be exported here.</p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img src={process.env.PUBLIC_URL + '/statics/gltf_share.png'} alt="gltf Share" className="gltfgraphic" />
                </div>
                <div className="image-description">Share your robot description in this software with developer teams to modify the same robot description</div>
            </div>
            <div className="description-container">
                <p>For understanding the capabilities of GLTF files in versioning your defined robot, refer to the <a href="https://roboeverything.com">documentation</a>.</p>
            </div>
        </div>
    );
}

export default ExportGLTF;