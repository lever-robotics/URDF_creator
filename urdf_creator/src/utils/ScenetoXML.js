import * as THREE from "three";
import { generateSensorXML } from "./generateSensorXML";

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene) => {
    let xml = `<robot name="GeneratedRobot">\n`;
    if (scene === undefined) return xml;

    // Helper to format vector as a string and flip y and z coordinates
    const formatVector = (vec) => `${vec.x} ${vec.z} ${vec.y}`;

    // Helper to convert rotation to URDF-compatible roll-pitch-yaw (rpy) and flip y and z
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
        if (node instanceof THREE.Mesh) {
            console.log(node.userData);
            const linkName = generateLinkName(node);
            linkIndex += 1;

            const position = formatVector(node.position);
            const rotation = quaternionToEuler(node.quaternion);

            // Start link
            xml += `  <link name="${linkName}">\n`;
            xml += `    <visual>\n`;
            xml += `      <origin xyz="0 0 0" rpy="0 0 0" />\n`;

            // Geometry
            const geometryType = node.geometry.type;
            let geometryXML = "";
            if (geometryType === "BoxGeometry") {
                const size = `${node.scale.x} ${node.scale.z} ${node.scale.y}`;
                geometryXML = `      <geometry>\n        <box size="${size}" />\n      </geometry>\n`;
            } else if (geometryType === "SphereGeometry") {
                const radius = node.scale.x / 3;
                geometryXML = `      <geometry>\n        <sphere radius="${radius}" />\n      </geometry>\n`;
            } else if (geometryType === "CylinderGeometry") {
                const radius = node.scale.x / 2; // Assume uniform scaling for the radius
                const height = node.scale.y;
                geometryXML = `      <geometry>\n        <cylinder radius="${radius}" length="${height}" />\n      </geometry>\n`;
            }
            xml += geometryXML;

            // Material
            if (node.material && node.material.color) {
                const color = node.material.color;
                xml += `      <material name="${node.material.name || "material"}">\n`;
                xml += `        <color rgba="${color.r} ${color.g} ${color.b} 1" />\n`;
                xml += `      </material>\n`;
            }

            // End visual
            xml += `    </visual>\n`;

            // Add collision element with the same geometry
            xml += `    <collision>\n`;
            xml += `      <origin xyz="0 0 0" rpy="0 0 0" />\n`;
            xml += geometryXML;
            xml += `    </collision>\n`;

            // Add inertial element
            const mass = node.userData.mass || 0;
            const { Ixx, Ixy, Ixz, Iyy, Iyz, Izz } = node.userData;
            xml += `    <inertial>\n`;
            xml += `      <origin xyz="0 0 0" rpy="0 0 0" />\n`;
            xml += `      <mass value="${mass}" />\n`;
            xml += `      <inertia ixx="${Ixx || 0}" ixy="${Ixy || 0}" ixz="${Ixz || 0}" iyy="${Iyy || 0}" iyz="${Iyz || 0}" izz="${Izz || 0}" />\n`;
            xml += `    </inertial>\n`;

            // Check for sensors and add Gazebo plugin if applicable
            if (node.userData.isSensor && node.userData.sensorType) {
                const sensorXML = generateSensorXML(node);
                xml += sensorXML;
            }

            // End link
            xml += `  </link>\n`;

            // Add joint if there's a parent link
            if (parentName) {
                xml += `  <joint name="${parentName}_to_${linkName}" type="fixed">\n`;
                xml += `    <parent link="${parentName}" />\n`;
                xml += `    <child link="${linkName}" />\n`;
                xml += `    <origin xyz="${position}" rpy="${rotation}" />\n`;
                xml += `  </joint>\n`;
            }

            // Recursively process children with the correct parent name
            node.children[0].children.forEach((child) => processNode(child, linkName));
        }
    };

    // Find the base node and start processing
    const baseNode = scene.children.find((child) => child.type === "Mesh");
    if (baseNode) {
        processNode(baseNode); // Initialize the recursion with the base node
    }

    // Close robot tag
    xml += `</robot>`;

    return xml;
};
