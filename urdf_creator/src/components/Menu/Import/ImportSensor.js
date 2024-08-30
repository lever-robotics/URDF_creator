import React from 'react';


// Create function that reads in and lists all the avalible gltf sensor files, picture and name
// Create function that reads in a specific gltf file name and adds it to the scene

const GltfFilesGrid = () => {
    const gltfFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFiles/myFile.gltf`;
    const imageFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFilesImages/myImage.png`;

    const fileNames = Object.keys(gltfFilePath);

    return (
        <div className="grid-container">
            {fileNames.map((fileName) => (
                <button key={fileName} className="gltf-button">
                    <img src={imageFilePath[fileName]} alt={fileName} />
                    <span>{fileName}</span>
                </button>
            ))}
        </div>
    );
};

export default GltfFilesGrid;
