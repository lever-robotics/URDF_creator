import React, { useContext } from 'react';
import JSZip from 'jszip';
import { saveAs } from 'file-saver';
import { URDFHistoryContext } from '../URDFContext/URDFHistoryContext';

const DownloadRobotPackage = () => {
    const { history, currentIndex } = useContext(URDFHistoryContext);

    const generateZip = async () => {
        const zip = new JSZip();

        // List of static files and their paths in the ZIP
        const filesToAdd = [
            { path: 'robot_package/config/example_config.yaml', zipPath: 'my_robot_description/config/example_config.yaml' },
            { path: 'robot_package/launch/example.launch.py', zipPath: 'my_robot_description/launch/example.launch.py' },
            { path: 'robot_package/rviz/my_robot.rviz', zipPath: 'my_robot_description/rviz/my_robot.rviz' },
            { path: 'robot_package/worlds/example.world', zipPath: 'my_robot_description/worlds/example.world' },
            { path: 'robot_package/CMakeLists.txt', zipPath: 'my_robot_description/CMakeLists.txt' },
            { path: 'robot_package/package.xml', zipPath: 'my_robot_description/package.xml' },
            { path: 'robot_package/README.md', zipPath: 'my_robot_description/README.md' }
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

        // Add the latest URDF file to the ZIP
        console.log('Current Index: ', currentIndex);
        console.log('History: ', history);
        if (currentIndex >= 0 && history[currentIndex]) {
            const urdfContent = history[currentIndex];
            zip.file('my_robot_description/urdf/example_robot.urdf', urdfContent);
        } else {
            console.error('No URDF file found in history.');
        }

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
