import React from 'react';
import JSZip from 'jszip';
import { saveAs } from 'file-saver';

const DownloadRobotPackage = () => {
    const generateZip = async () => {
        const zip = new JSZip();

        // List of static files and their paths in the ZIP
        const filesToAdd = [
            { path: 'robot_package/config/example_config.yaml', zipPath: 'robot_package/config/example_config.yaml' },
            { path: 'robot_package/urdf/example_robot.urdf', zipPath: 'robot_package/urdf/example_robot.urdf' },
            { path: 'robot_package/launch/example.launch', zipPath: 'robot_package/launch/example.launch' },
            { path: 'robot_package/worlds/example.world', zipPath: 'robot_package/worlds/example.world' },
            { path: 'robot_package/CMakeLists.txt', zipPath: 'robot_package/CMakeLists.txt' },
            { path: 'robot_package/package.xml', zipPath: 'robot_package/package.xml' },
            { path: 'robot_package/README.md', zipPath: 'robot_package/README.md' }
        ];

        // Function to fetch and add files to the zip
        const addFilesToZip = async (fileInfo) => {
            const response = await fetch(`${process.env.PUBLIC_URL}/${fileInfo.path}`);
            const content = await response.blob();
            zip.file(fileInfo.zipPath, content);
        };

        // Add all files to the ZIP
        const filePromises = filesToAdd.map(fileInfo => addFilesToZip(fileInfo));
        await Promise.all(filePromises);

        // Generate the ZIP file and trigger the download
        zip.generateAsync({ type: "blob" }).then(function (content) {
            saveAs(content, "robot_description_package.zip");
        });
    };

    return (
        <div>
            <button onClick={generateZip}>Download Robot Description Package</button>
        </div>
    );
};

export default DownloadRobotPackage;
