// Handle misc download types
import * as openDB from "idb";
import { GLTFExporter } from "three/addons/exporters/GLTFExporter.js";
import JSZip from "jszip";
import { saveAs } from "file-saver";
import { ScenetoXML } from "./ScenetoXML";
import { ScenetoSDF } from "./ScenetoSDF";
import { LaunchPropertiesContained } from "./CreatePackage/LaunchPropertiesContained";
import { GenerateLaunchFile } from "./CreatePackage/GenerateLaunchFile";
import { GeneratePackageXMLFile, GenerateCMakelistsFile } from "./CreatePackage/GenerateBuildFiles";

export async function handleDownload(scene, type, title) {
    if (type === "urdfpackage") {
        const urdf = ScenetoXML(scene, title.replace(" ", "_"));
        const sdf = ScenetoSDF(scene, title.replace(" ", "_"));
        const projectProperties = LaunchPropertiesContained(scene); // Function that returns array of which sensors are used so it can configure the launch file
        await generateZip(urdf, sdf, projectProperties, title.replace(" ", "_"));
    } else if (type === "urdf") {
        const urdf = ScenetoXML(scene, title.replace(" ", "_"));
        otherFileDownload(urdf, type, title.replace(" ", "_"));
    } else if (type === "gltf") {
        const exporter = new GLTFExporter();
        exporter.parse(scene, (gltf) => {
            otherFileDownload(JSON.stringify(gltf), type, title.replace(" ", "_"));
        });
        // const json = scene.toJSON();
        // otherFileDownload(JSON.stringify(json), type, title);
    } else {
        // Probably should implement an error box
    }
}

export function otherFileDownload(data, type, title) {
    // Create Blob
    const blob = new Blob([data], { type: `application/${type}` });

    // Create a link element and set href to blob
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = `${title}.${type}`;

    // Append the file, click it, then remove it
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);

    // Free up resources
    URL.revokeObjectURL(link.href);
}

export async function generateZip(urdfContent, SDFContent, projectProperties, title) {
    const zip = new JSZip();

    // List of static files and their paths in the ZIP
    const filesToAdd = [
        {
            path: "robot_package/config/example_config.yaml",
            zipPath: `${title}_description/config/example_config.yaml`,
        },
        {
            path: "robot_package/rviz/my_robot.rviz",
            zipPath: `${title}_description/rviz/my_robot.rviz`,
        },
        {
            path: "robot_package/worlds/example.world",
            zipPath: `${title}_description/worlds/example.world`,
        },
        {
            path: "robot_package/README.md",
            zipPath: `${title}_description/README.md`,
        },
    ];
    //generate the package.xml file
    zip.file(`${title}_description/package.xml`, GeneratePackageXMLFile(title, projectProperties));
    //generate the CMakeLists.txt file
    zip.file(`${title}_description/CMakeLists.txt`, GenerateCMakelistsFile(title, projectProperties));

    // Function to fetch and add files to the zip
    const addFilesToZip = async (fileInfo) => {
        const response = await fetch(`/${fileInfo.path}`);
        const content = await response.blob();
        zip.file(fileInfo.zipPath, content);
    };

    // Add all files to the ZIP
    const filePromises = filesToAdd.map((fileInfo) => addFilesToZip(fileInfo));
    await Promise.all(filePromises);

    //Add the meshes to the ZIP
    // const db = await openDB("stlFilesDB", 1);
    // const files = await db.getAll("files");
    // files.forEach(async (file) => {
    //     zip.file(`${title}_description/meshes/${file.name}`, file.file);
    // });

    // Add the URDF file to the ZIP
    if (urdfContent) {
        zip.file(`${title}_description/urdf/${title}.urdf`, urdfContent);
    } else {
        console.error("No URDF file found in the state.");
    }

    // Add the SDF file to the ZIP
    if (SDFContent) {
        zip.file(`${title}_description/model/${title}.sdf`, SDFContent);
    }

    //Programatically generate the launch file
    const launchFileContent = GenerateLaunchFile(title, projectProperties);

    // Add the launch file to the ZIP
    zip.file(`${title}_description/launch/${title}.launch.py`, launchFileContent);

    // Generate the ZIP file and trigger the download
    zip.generateAsync({ type: "blob" }).then(function (content) {
        saveAs(content, `${title}_package.zip`);
    });
}
