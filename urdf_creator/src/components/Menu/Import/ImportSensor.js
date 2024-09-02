import React from 'react';

// Define the list of .gltf file names (without extensions)
const fileNames = [
    'robot',
    // Add more file names as needed
];

const GltfFile = ({ fileName }) => {
    const gltfFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFiles/${fileName}.gltf`;
    const imageFilePath = `${process.env.PUBLIC_URL}/statics/GLTFFilesImages/${fileName}.png`;

    const addGltfObject = (gltfFile) => {
        // Read the .gltf file
        const reader = new FileReader();
        reader.onload = (event) => {
            const gltfData = event.target.result;

            // Use the gltfData to create the URDF object
            const urdfObject = createUrdfObject(gltfData);

            // Add the URDF object to the scene
            addToScene(urdfObject);
        };
        reader.readAsText(gltfFile);
    };

    return (
        <button className="gltf-button" onClick={() => addGltfObject(gltfFilePath)}>
            <img src={imageFilePath} alt={fileName} />
            <span>{fileName}</span>
        </button>
    );
};

const GltfFilesGrid = () => {
    const gltfButtons = fileNames.map((fileName) => (
        <GltfFile key={fileName} fileName={fileName} />
    ));

    return <div className="grid-container">{gltfButtons}</div>;
};

export default GltfFilesGrid;
