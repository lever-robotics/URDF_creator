// Handle misc download types
import * as THREE from "three";
import { GLTFExporter } from "three/addons/exporters/GLTFExporter.js";
import JSZip from "jszip";
import { saveAs } from "file-saver";
import { ScenetoXML } from "./ScenetoXML";
import { ScenetoSDF } from "./ScenetoSDF";
import { SensorsContained } from "./CreatePackage/SensorsContained";

export async function handleDownload(scene, type, title) {
  if (type === "urdf") {
    const urdf = ScenetoXML(scene);
    const sdf = ScenetoSDF(scene);
    const sensorsContained = SensorsContained(scene); // Function that returns array of which sensors are used so it can configure the launch file
    await generateZip(urdf, sdf, sensorsContained, title);
  } else if (type === "gltf") {
    const exporter = new GLTFExporter();
    exporter.parse(scene, (gltf) => {
      otherFileDownload(JSON.stringify(gltf), type, title);
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

export async function generateZip(urdfContent, SDFContent, title) {
  const zip = new JSZip();

  // List of static files and their paths in the ZIP
  const filesToAdd = [
    {
      path: "robot_package/config/example_config.yaml",
      zipPath: `${title}/config/example_config.yaml`,
    },
    {
      path: "robot_package/launch/example.launch.py",
      zipPath: `${title}/launch/robot_sim.launch.py`,
    },
    {
      path: "robot_package/rviz/my_robot.rviz",
      zipPath: `${title}/rviz/my_robot.rviz`,
    },
    {
      path: "robot_package/worlds/example.world",
      zipPath: `${title}/worlds/example.world`,
    },
    {
      path: "robot_package/CMakeLists.txt",
      zipPath: `${title}/CMakeLists.txt`,
    },
    {
      path: "robot_package/package.xml",
      zipPath: `${title}/package.xml`,
    },
    {
      path: "robot_package/README.md",
      zipPath: `${title}/README.md`,
    },
  ];

  // Function to fetch and add files to the zip
  const addFilesToZip = async (fileInfo) => {
    const response = await fetch(`${process.env.PUBLIC_URL}/${fileInfo.path}`);
    const content = await response.blob();
    zip.file(fileInfo.zipPath, content);
  };

  // Add all files to the ZIP
  const filePromises = filesToAdd.map((fileInfo) => addFilesToZip(fileInfo));
  await Promise.all(filePromises);

  // Add the latest URDF file to the ZIP
  if (urdfContent) {
    zip.file(`${title}/urdf/${title}.urdf`, urdfContent);
  } else {
    console.error("No URDF file found in the state.");
  }
  if (SDFContent) {
    zip.file(`${title}/model/${title}.sdf`, SDFContent);
  }

  //Programatically generate the launch file
  // const launchFileContent = GenerateLaunchFileContent(title);

  // Generate the ZIP file and trigger the download
  zip.generateAsync({ type: "blob" }).then(function (content) {
    saveAs(content, `${title}_package.zip`);
  });
}
