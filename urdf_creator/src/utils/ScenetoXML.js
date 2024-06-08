import * as THREE from "three";
import { generateSensorXML } from "./generateSensorXML";
import findBaseLink from "./findBaseLink";
import urdfObject from "../Models/urdfObject";

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene) => {
    let xml = `<robot name="GeneratedRobot">\n`;
    if (scene === undefined) return xml;

    // Helper to flip THREE xyz into urdf zxy
    // Helper to flip THREE xyz into urdf zxy
    const formatVector = (vec) => `${vec.z} ${vec.x} ${vec.y}`;

    // Helper to convert rotation to URDF-compatible roll-pitch-yaw (rpy) xyz --> zxy
    // Helper to convert rotation to URDF-compatible roll-pitch-yaw (rpy) xyz --> zxy
    const quaternionToEuler = (quaternion) => {
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quaternion, "XYZ");
        return `${euler.y} ${euler.x} ${euler.z}`;
    };

    function quaternionToRPY(quaternion) {
        // Extract the quaternion components
        const { x, y, z, w } = quaternion;

        // Calculate the roll (x-axis rotation)
        const sinr_cosp = 2 * (w * x + y * z);
        const cosr_cosp = 1 - 2 * (x * x + y * y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);

        // Calculate the pitch (y-axis rotation)
        const sinp = Math.sqrt(1 + 2 * (w * y - x * z));
        const cosp = Math.sqrt(1 - 2 * (w * y - x * z));
        const pitch = 2 * Math.atan2(sinp, cosp) - Math.PI / 2;

        // Calculate the yaw (z-axis rotation)
        const siny_cosp = 2 * (w * z + x * y);
        const cosy_cosp = 1 - 2 * (y * y + z * z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        return `${yaw} ${pitch} ${roll}`;
    }

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

            const offset = formatVector(node.link.position);
            const rotation = quaternionToRPY(node.quaternion);

            // Start link
            xml += `  <link name="${linkName}">\n`;
            xml += `    <visual>\n`;
            xml += `      <origin xyz="${offset}" rpy="0 0 0" />\n`;

            // Geometry
            const geometryType = node.mesh.geometry.type;
            let geometryXML = "";
            if (geometryType === "BoxGeometry") {
                const size = `${formatVector(node.mesh.scale)}`;
                geometryXML = `      <geometry>\n        <box size="${size}" />\n      </geometry>\n`;
            } else if (geometryType === "SphereGeometry") {
                const radius = node.mesh.scale.x / 2;
                geometryXML = `      <geometry>\n        <sphere radius="${radius}" />\n      </geometry>\n`;
            } else if (geometryType === "CylinderGeometry") {
                const radius = node.mesh.scale.x / 2; // Assume uniform scaling for the radius
                const height = node.mesh.scale.y;
                geometryXML = `      <geometry>\n        <cylinder radius="${radius}" length="${height}" />\n      </geometry>\n`;
            }
            xml += geometryXML;

            // Material
            if (node.mesh.material && node.mesh.material.color) {
                const color = node.mesh.material.color;
                xml += `      <material name="${node.mesh.material.name || "material"}">\n`;
                xml += `        <color rgba="${color.r} ${color.g} ${color.b} 1" />\n`;
                xml += `      </material>\n`;
            }

            // End visual
            xml += `    </visual>\n`;

            // Add collision element with the same geometry
            xml += `    <collision>\n`;
            xml += `      <origin xyz="${offset}" rpy="0 0 0" />\n`;
            xml += geometryXML;
            xml += `    </collision>\n`;

            // Add inertial element
            const mass = node.userData.inertia.mass || 0;
            const { ixx, ixy, ixz, iyy, iyz, izz } = node.userData.inertia;
            xml += `    <inertial>\n`;
            xml += `      <origin xyz="${offset}" rpy="0 0 0" />\n`;
            xml += `      <mass value="${mass}" />\n`;
            xml += `      <inertia ixx="${ixx || 0}" ixy="${ixy || 0}" ixz="${ixz || 0}" iyy="${iyy || 0}" iyz="${iyz || 0}" izz="${izz || 0}" />\n`;
            xml += `    </inertial>\n`;

            // Check for sensors and add Gazebo plugin if applicable
            if (node.userData.sensor) {
                const sensorXML = generateSensorXML(node);
                xml += sensorXML;
            }

            // End link
            xml += `  </link>\n`;

            // Add joint if there's a parent link
            if (parentName) {
                xml += `  <joint name="${parentName}_to_${linkName}" type="${node.joint.type}">\n`;
                xml += `    <parent link="${parentName}" />\n`;
                xml += `    <child link="${linkName}" />\n`;

                // because urdf is dumb, children links are connected to parent joints, not parent meshes
                // this code accounts for that and sets the joint origin in relation to the parent's joint origin
                // ie it add the links position to its own since it isnt passed with
                const originInRelationToParentsJoint = new THREE.Vector3();
                originInRelationToParentsJoint.copy(node.position);
                originInRelationToParentsJoint.add(node.parent.position);

                xml += `    <origin xyz="${formatVector(originInRelationToParentsJoint)}" rpy="${rotation}" />\n`;
                if (node.joint.type !== "fixed") {
                    const quaternion = new THREE.Quaternion();
                    quaternion.setFromEuler(node.joint.rotation);
                    const newAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
                    xml += `    <axis xyz="${formatVector(newAxis)}"/>\n`;
                    if (node.joint.type !== "continuous") {
                        xml += `    <limit effort="1000.0" lower="${node.joint.min}" upper="${node.joint.max}" velocity="0.5"/>`;
                    }
                }
                xml += `  </joint>\n`;
            }

            // Recursively process children with the correct parent name
            node.getChildren().forEach((child) => processNode(child, linkName));
        }
    };

    // Find the base node and start processing
    const baseLink = findBaseLink(scene);
    if (baseLink) processNode(baseLink);

    // Close robot tag
    xml += `</robot>`;

    return xml;
};
