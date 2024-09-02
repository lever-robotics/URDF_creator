import React from 'react';
import './ImportSensor.css';
import './STLImport.css';
// Define the list of .gltf file names (without extensions)
const fileNames = [
    { fileName: 'realsense', displayName: 'RealSense' },
    // Add more file names as needed
];

const GltfFile = ({ fileName, displayName, loadSingleObject }) => {
    const gltfFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFiles/${fileName}.gltf`;
    const imageFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFilesImages/${fileName}.png`;

    const loadGltfFile = () => {
        loadSingleObject(gltfFilePath);
    };

    return (
        <button className="gltf-button" onClick={loadGltfFile}>
            <img src={imageFilePath} alt={fileName} className='sensor-graphic' />
            <span>{displayName}</span>
        </button>
    );
};

const GltfFilesGrid = ({loadSingleObject}) => {
    const gltfButtons = fileNames.map((file) => (
        <GltfFile key={file.fileName} fileName={file.fileName} displayName={file.displayName} loadSingleObject={loadSingleObject} />
    ));

    return (
        <div>
            <div className="header">
                <div className="stl-import-title">
            <h3>Add sensor to your project</h3>
        </div>
        </div>
            <div className="grid-container">
        {gltfButtons}
        </div>
    </div>
    
    );
};

export default GltfFilesGrid;
