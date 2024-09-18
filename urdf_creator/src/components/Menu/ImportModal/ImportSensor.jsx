import React from 'react';
import './ImportSensor.css';
import './Import.css';
// Define the list of .gltf file names (without extensions)
const fileNames = [
    { fileName: 'realsense', displayName: 'RealSense' },
    { fileName: 'rsplidar', displayName: 'RPLidar' },
    { fileName: 'imu_sensor', displayName: 'IMU Sensor' },
    // Add more file names as needed
];

const GltfFile = ({ fileName, displayName, handleSensorClick }) => {
    const gltfFilePath = `/statics/GLTFFiles/${fileName}.gltf`;
    const imageFilePath = `/statics/GLTFFilesImages/${fileName}.png`;

    const loadGltfFile = () => {
        handleSensorClick(gltfFilePath);
    };

    return (
        <button className="gltf-button" onClick={loadGltfFile}>
            <img src={imageFilePath} alt={fileName} className='sensor-graphic' />
            <span>{displayName}</span>
        </button>
    );
};

const GltfFilesGrid = ({handleSensorClick}) => {
    const gltfButtons = fileNames.map((file) => (
        <GltfFile key={file.fileName} fileName={file.fileName} displayName={file.displayName} handleSensorClick={handleSensorClick} />
    ));

    return (
        <div>
            <div className="header">
                <div className="import-title">
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
