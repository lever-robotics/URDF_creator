import * as THREE from "three";
import { generateSensorXML } from "./generateSensorXML.js";
import findRootFrame from "./findRootFrame.js";
import Frame from "../Models/Frame.jsx";
import { quaternionToRPY } from "./quaternionToRPY.js";

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene: THREE.Object3D, projectTitle: string) => {
    let xml = `<robot name="${projectTitle.replace(" ", "_")}">\n`;
    if (scene === undefined) {
        xml += `</robot>`;
        return xml;
    }

    // Helper to flip THREE xyz into urdf zxy
    const formatVector = (vec: THREE.Vector3) => `${vec.x} ${vec.y} ${vec.z}`;

    // Variables to keep track of link naming
    let linkIndex = 0;
    const generateLinkName = (node: Frame) => {
        return node.name || (linkIndex === 0 ? "base_link" : `link${linkIndex}`);
    };

    // Function to process a single node
    const processNode = (node: Frame, parentName: string | null = null) => {
        if (node instanceof Frame) {
            const linkName = generateLinkName(node);
            linkIndex += 1;
            // debugger;
            let offset = formatVector(node.link!.position);
            let rotation = quaternionToRPY(node.quaternion);
            let linkRotation = "0 0 0";

            if (node.isRootFrame) {
                offset = formatVector(node.position);
                linkRotation = quaternionToRPY(node.quaternion);
            } else if (node.parentFrame!.isRootFrame) {
                const quaternion = new THREE.Quaternion();
                rotation = quaternionToRPY(quaternion.multiplyQuaternions(node.parentFrame!.quaternion, node.quaternion));
            }

            // Start link
            xml += `  <link name="${linkName}">\n`;
            xml += `    <visual>\n`;
            xml += `      <origin xyz="${offset}" rpy="${linkRotation}" />\n`;

            // Geometry
            const geometryType = node.mesh!.geometry.type;
            let geometryXML = "";
            if (geometryType === "BoxGeometry") {
                const size = `${formatVector(node.objectScale)}`;
                geometryXML = `      <geometry>\n        <box size="${size}" />\n      </geometry>\n`;
            } else if (geometryType === "SphereGeometry") {
                const radius = node.objectScale.x / 2;
                geometryXML = `      <geometry>\n        <sphere radius="${radius}" />\n      </geometry>\n`;
            } else if (geometryType === "CylinderGeometry") {
                const radius = node.objectScale.x / 2; // Assume uniform scaling for the radius
                const height = node.objectScale.z;
                geometryXML = `      <geometry>\n        <cylinder radius="${radius}" length="${height}" />\n      </geometry>\n`;
            }
            xml += geometryXML;

            // Material
            if (node.mesh!.material && node.mesh!.material.color) {
                const color = node.mesh!.material.color;
                xml += `      <material name="${node.mesh!.material.name || node.name + "-material"}">\n`;
                xml += `        <color rgba="${color.r} ${color.g} ${color.b} 1" />\n`;
                xml += `      </material>\n`;
            }

            // End visual
            xml += `    </visual>\n`;

            // Add collision element with the same geometry
            xml += `    <collision>\n`;
            xml += `      <origin xyz="${offset}" rpy="${linkRotation}" />\n`;
            xml += geometryXML;
            xml += `    </collision>\n`;

            // Add inertial element
            const mass = node.inertia!.mass || 0;
            const { ixx, ixy, ixz, iyy, iyz, izz } = node.inertia!;
            xml += `    <inertial>\n`;
            xml += `      <origin xyz="${offset}" rpy="${linkRotation}" />\n`;
            xml += `      <mass value="${mass}" />\n`;
            xml += `      <inertia ixx="${ixx || 0}" ixy="${ixy || 0}" ixz="${ixz || 0}" iyy="${iyy || 0}" iyz="${iyz || 0}" izz="${izz || 0}" />\n`;
            xml += `    </inertial>\n`;

            // Check for sensors and add Gazebo plugin if applicable
            if (node.sensor!.type) {
                const sensorXML = generateSensorXML(node);
                xml += sensorXML;
            }

            // End link
            xml += `  </link>\n`;

            // Add joint if there's a parent link
            if (parentName) {
                xml += `  <joint name="${parentName}_to_${linkName}" type="${node.jointType}">\n`;
                xml += `    <parent link="${parentName}" />\n`;
                xml += `    <child link="${linkName}" />\n`;

                // because urdf is dumb, children links are connected to parent joints, not parent meshes
                // this code accounts for that and sets the joint origin in relation to the parent's joint origin
                // ie it add the links position to its own since it isnt passed with
                const originInRelationToParentsJoint = new THREE.Vector3();
                originInRelationToParentsJoint.copy(node.position);
                originInRelationToParentsJoint.add(node.parentFrame!.link!.position);

                if (node.parentFrame!.isRootFrame) {
                    node.getWorldPosition(originInRelationToParentsJoint);
                }

                xml += `    <origin xyz="${formatVector(originInRelationToParentsJoint)}" rpy="${rotation}" />\n`;
                if (node.jointType !== "fixed") {
                    const quaternion = new THREE.Quaternion();
                    quaternion.setFromEuler(node.axis!.rotation);
                    const newAxis = new THREE.Vector3(...node.axis!.axis).applyQuaternion(quaternion);
                    xml += `    <axis xyz="${formatVector(newAxis)}"/>\n`;
                    if (node.jointType !== "continuous") {
                        xml += `    <limit effort="1000.0" lower="${node.min}" upper="${node.max}" velocity="0.5"/>`;
                    }
                }
                xml += `  </joint>\n`;
            }

            // Recursively process children with the correct parent name
            node.getFrameChildren().forEach((child) => processNode(child, linkName));
        }
    };

    // Find the base node and start processing
    const rootFrame = findRootFrame(scene);
    if (rootFrame) processNode(rootFrame);

    // Close robot tag
    xml += `</robot>`;

    return xml;
};
