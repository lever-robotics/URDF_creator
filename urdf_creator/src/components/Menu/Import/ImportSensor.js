import React from 'react';

// Define the list of .gltf file names (without extensions)
const fileNames = [
    'robot',
    // Add more file names as needed
];

const GltfFilesGrid = () => {
    return (
        <div className="grid-container">
            {fileNames.map((fileName) => {
                const gltfFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFiles/${fileName}.gltf`;
                const imageFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFilesImages/${fileName}.png`;

                return (
                    <button key={fileName} className="gltf-button">
                        <img src={imageFilePath} alt={fileName} />
                        <span>{fileName}</span>
                    </button>
                );
            })}
        </div>
    );
};

export default GltfFilesGrid;