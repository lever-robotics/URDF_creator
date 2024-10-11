import { saveAs } from "file-saver";
// Handle misc download types
import { openDB } from "idb";
import JSZip from "jszip";
import type * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter";
import type ThreeScene from "../components/ThreeDisplay/ThreeScene";
import {
    GenerateCMakelistsFile,
    GeneratePackageXMLFile,
    GenerateSimCMakelistsFile,
    GenerateSimPackageXMLFile,
} from "./CreatePackage/GenerateBuildFiles";
import {
    GenerateRealLaunchFile,
    GenerateSimLaunchFile,
} from "./CreatePackage/GenerateLaunchFile";
import { LaunchPropertiesContained } from "./CreatePackage/LaunchPropertiesContained";
import { ScenetoSDF } from "./ScenetoSDF";
import { ScenetoXML } from "./ScenetoXML";

export async function handleDownload(
    scene: ThreeScene,
    type: string,
    title: string,
) {
    // export the robot descrition pacakge containing the URDF
    if (type === "urdfpackage") {
        const urdf = ScenetoXML(scene, title);
        const projectProperties = LaunchPropertiesContained(scene); // Function that returns array of which sensors are used so it can configure the launch file
        await generateURDFZip(urdf, title);
        // export the gazebo simulation package containing the SDF
    } else if (type === "gazebopackage") {
        const sdf = ScenetoSDF(scene, title);
        const urdf = ScenetoXML(scene, title);
        const projectProperties = LaunchPropertiesContained(scene); // Function that returns array of which sensors are used so it can configure the launch file
        await generateGazeboZip(sdf, urdf, title);
    } else if (type === "urdf") {
        const urdf = ScenetoXML(scene, title);
        otherFileDownload(urdf, type, title);
    } else if (type === "gltf") {
        const exporter = new GLTFExporter();
        exporter.parse(
            scene.scene,
            (gltf) => {
                otherFileDownload(JSON.stringify(gltf), type, title);
            },
            (error) => {
                console.error("An error occurred during GLTF export:", error);
            },
        );
        // const json = scene.toJSON();
        // otherFileDownload(JSON.stringify(json), type, title);
    } else {
        // Probably should implement an error box
        console.error("Invalid download type");
    }
}

export function otherFileDownload(data: string, type: string, title: string) {
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

export async function GenerateZip(
    files: { zipPath: string; content: Blob | string }[],
    title: string,
    include_meshes: boolean,
) {
    const zip = new JSZip();

    // Loop over the files to add them to the zip
    for (const file of files) {
        zip.file(file.zipPath, file.content);
    }

    //Add the meshes to the ZIP if its been asked to be included. So when ever the robot_description package is created
    if (include_meshes) {
        const db = await openDB("stlFilesDB", 1);
        const mesh_files = await db.getAll("files");
        for (const file of mesh_files) {
            zip.file(`${title}_description/meshes/${file.name}`, file.file);
        }
    }

    // Generate the zip file and trigger the download
    zip.generateAsync({ type: "blob" }).then((content) => {
        saveAs(content, `${title}_package.zip`);
    });
}

export async function GenerateDescriptionFiles(
    urdfContent: string,
    title: string,
) {
    // Prepare file list for robot_description folder
    const descriptionFiles = [
        {
            zipPath: `${title}_description/config/example_config.yaml`,
            content: await fetchFileContent(
                "robot_package/config/example_config.yaml",
            ),
        },
        {
            zipPath: `${title}_description/rviz/my_robot.rviz`,
            content: await fetchFileContent("robot_package/rviz/my_robot.rviz"),
        },
        {
            zipPath: `${title}_description/package.xml`,
            content: GeneratePackageXMLFile(title),
        },
        {
            zipPath: `${title}_description/CMakeLists.txt`,
            content: GenerateCMakelistsFile(title),
        },
    ];

    // Add the URDF file if provided
    if (urdfContent) {
        descriptionFiles.push({
            zipPath: `${title}_description/urdf/${title}.urdf`,
            content: urdfContent,
        });
    }

    // Add launch file
    descriptionFiles.push({
        zipPath: `${title}_description/launch/${title}.launch.py`,
        content: GenerateRealLaunchFile(title),
    });

    return descriptionFiles;
}

export async function GenerateGazeboFiles(sdfContent: string, title: string) {
    // Prepare file list for robot_gazebo folder
    const gazeboFiles = [
        {
            zipPath: `${title}_gazebo/worlds/example.world`,
            content: await fetchFileContent(
                "robot_gazebo/worlds/example.world",
            ),
        },
        {
            zipPath: `${title}_gazebo/package.xml`,
            content: GenerateSimPackageXMLFile(title),
        },
        {
            zipPath: `${title}_gazebo/CMakeLists.txt`,
            content: GenerateSimCMakelistsFile(title),
        },
    ];

    // Add SDF file if provided
    if (sdfContent) {
        gazeboFiles.push({
            zipPath: `${title}_gazebo/model/${title}.sdf`,
            content: sdfContent,
        });
    }

    // Add launch file
    gazeboFiles.push({
        zipPath: `${title}_gazebo/launch/${title}.launch.py`,
        content: GenerateSimLaunchFile(title),
    });

    return gazeboFiles;
}

export async function generateURDFZip(urdfContent: string, title: string) {
    const descriptionFiles = await GenerateDescriptionFiles(urdfContent, title);
    await GenerateZip(descriptionFiles, title, true);
}

export async function generateGazeboZip(
    sdfContent: string,
    urdfContent: string,
    title: string,
) {
    const descriptionFiles = await GenerateDescriptionFiles(urdfContent, title);
    const gazeboFiles = await GenerateGazeboFiles(sdfContent, title);

    // Combine both the description and gazebo files
    const allFiles = [...descriptionFiles, ...gazeboFiles];
    await GenerateZip(allFiles, title, true);
}

// Utility function to fetch file content via URL
async function fetchFileContent(path: string): Promise<Blob> {
    const response = await fetch(`/${path}`);
    return await response.blob();
}
