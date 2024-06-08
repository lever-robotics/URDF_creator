import * as THREE from "three";
import { generateSensorSDF } from "./generateSensorSDF";
import findBaseLink from "./findBaseLink";
import urdfObject from "../Models/urdfObject";

// Helper function to convert Scene to SDF-compatible XML
export const ScenetoSDF = (scene) => {
    debugger;
    let xml = `<sdf version="1.6">\n`;
    if (scene === undefined) return xml;

    xml += `<model name="GeneratedModel" canonical_link='base_link'>\n`;

    // Helper to format vector as a string and flip y and z coordinates
    const formatVector = (vec) => `${vec.x} ${vec.z} ${vec.y}`;

    // Helper to convert rotation to SDF-compatible roll-pitch-yaw (rpy) and flip y and z
    const quaternionToEuler = (quaternion) => {
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quaternion, "XYZ");
        return `${euler.x} ${euler.z} ${euler.y}`;
    };

    // Variables to keep track of link naming
    let linkIndex = 0;
    const generateLinkName = (node) => {
        return node.userData.name || (linkIndex === 0 ? "base_link" : `link${linkIndex}`);
    };

    // Function to process a single node
    const processNode = (node, parentName = null) => {
        if (node instanceof urdfObject) {
            const linkName = generateLinkName(node);
            linkIndex += 1;

            const position = formatVector(node.position);
            const offset = formatVector(node.link.position);
            const rotation = quaternionToEuler(node.quaternion);

            // Start link
            xml += `  <link name="${linkName}">\n`;
            xml += `    <pose>${position} ${rotation}</pose>\n`;

            // Geometry
            const geometryType = node.mesh.geometry.type;
            let geometryXML = "";
            if (geometryType === "BoxGeometry") {
                const size = `${node.mesh.scale.x} ${node.mesh.scale.z} ${node.mesh.scale.y}`;
                geometryXML = `    <collision name='${node.userData.name} collision'>\n      <geometry>\n        <box>\n          <size>${size}</size>\n        </box>\n      </geometry>\n    </collision>\n`;
                geometryXML += `    <visual name='${node.userData.name} visual'>\n      <geometry>\n        <box>\n          <size>${size}</size>\n        </box>\n      </geometry>\n    </visual>\n`;
            } else if (geometryType === "SphereGeometry") {
                const radius = node.mesh.scale.x / 3;
                geometryXML = `    <collision name='${node.userData.name} collision'>\n      <geometry>\n        <sphere>\n          <radius>${radius}</radius>\n        </sphere>\n      </geometry>\n    </collision>\n`;
                geometryXML += `    <visual name='${node.userData.name} visual'>\n      <geometry>\n        <sphere>\n          <radius>${radius}</radius>\n        </sphere>\n      </geometry>\n    </visual>\n`;
            } else if (geometryType === "CylinderGeometry") {
                const radius = node.mesh.scale.x / 2; // Assume uniform scaling for the radius
                const height = node.mesh.scale.y;
                geometryXML = `    <collision name='${node.userData.name} collision'>\n      <geometry>\n        <cylinder>\n          <radius>${radius}</radius>\n          <length>${height}</length>\n        </cylinder>\n      </geometry>\n    </collision>\n`;
                geometryXML += `    <visual name='${node.userData.name} visual'>\n      <geometry>\n        <cylinder>\n          <radius>${radius}</radius>\n          <length>${height}</length>\n        </cylinder>\n      </geometry>\n    </visual>\n`;
            }
            xml += geometryXML;

            // Material
            if (node.mesh.material && node.mesh.material.color) {
                const color = node.mesh.material.color;
                xml += `    <material>\n      <script>\n        <uri>file://media/materials/scripts/gazebo.material</uri>\n        <name>${node.mesh.material.name || "Gazebo/Red"}</name>\n      </script>\n      <ambient>${color.r} ${color.g} ${color.b} 1</ambient>\n      <diffuse>${color.r} ${color.g} ${color.b} 1</diffuse>\n      <specular>0.1 0.1 0.1 1</specular>\n      <emissive>0 0 0 1</emissive>\n    </material>\n`;
            }

            // Add inertial element
            const mass = node.userData.inertia.mass || 0;
            const { ixx, ixy, ixz, iyy, iyz, izz } = node.userData.inertia;
            xml += `    <inertial>\n      <mass>${mass}</mass>\n      <inertia>\n        <ixx>${ixx || 0}</ixx>\n        <ixy>${ixy || 0}</ixy>\n        <ixz>${ixz || 0}</ixz>\n        <iyy>${iyy || 0}</iyy>\n        <iyz>${iyz || 0}</iyz>\n        <izz>${izz || 0}</izz>\n      </inertia>\n    </inertial>\n`;

            // Check for sensors and add Gazebo plugin if applicable
            if (node.userData.sensor) {
                const sensorSDF = generateSensorSDF(node);
                xml += sensorSDF;
            }

            // End link
            xml += `  </link>\n`;

            // Add joint if there's a parent link
            if (parentName) {
                xml += `  <joint name="${parentName}_to_${linkName}" type="${node.joint.type}">\n`;
                xml += `    <parent>${parentName}</parent>\n`;
                xml += `    <child>${linkName}</child>\n`;
                xml += `    <pose>${position} ${rotation}</pose>\n`;
                xml += `  </joint>\n`;
            }

            // Recursively process children with the correct parent name
            node.getChildren().forEach((child) => processNode(child, linkName));
        }
    };

    // Find the base node and start processing
    const baseLink = findBaseLink(scene);
    if (baseLink) processNode(baseLink);

    // Close model and sdf tags
    xml += `</model>\n`;
    xml += `</sdf>`;

    return xml;
};
