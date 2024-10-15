import * as THREE from "three";
import Frame from "../Models/Frame";
import type ThreeScene from "../components/ThreeDisplay/ThreeScene";
import findRootFrame from "./findRootFrame";
import { generateSensorXML } from "./generateSensorXML";
import { quaternionToRPY } from "./quaternionToRPY";

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene: ThreeScene, projectTitle: string) => {
    let xml = `<robot name="${projectTitle.replace(" ", "_")}">\n`;
    if (scene === undefined) {
        xml += "</robot>";
        return xml;
    }

    // World Frame Link
    const worldFrame = scene.worldFrame;
    xml += `  <link name="${worldFrame.name}"></link>\n`;

    // Variables to keep track of link naming
    let linkIndex = 0;
    const generateLinkName = (node: Frame) => {
        return (
            node.name || (linkIndex === 0 ? "base_link" : `link${linkIndex}`)
        );
    };

    // Function to process a single node
    const processNode = (node: Frame, parentName: string) => {
        if (node instanceof Frame) {
            const linkName = generateLinkName(node);
            linkIndex += 1;

            // Add Joint then Link info
            xml += generateJoint(node, linkName, parentName);
            xml += generateLink(node, linkName, projectTitle);

            // Recursively process children with the correct parent name
            for (const child of node.getFrameChildren()) {
                processNode(child, linkName);
            }
        }
    };

    // Find the base node and start processing
    if (scene.rootFrame) processNode(scene.rootFrame, worldFrame.name);

    // Close robot tag
    xml += "</robot>";

    return xml;
};

// Helper to flip THREE xyz into urdf zxy
function formatVector(vec: THREE.Vector3): string {
    return `${vec.x} ${vec.y} ${vec.z}`;
}

/**
 *
 * @param node
 * @param linkName
 * @param projectTitle
 * @returns The XML for a single Link, collision, visual, and inertial data included
 */
function generateLink(
    node: Frame,
    linkName: string,
    projectTitle: string,
): string {
    // I didn't want to delete this code even though we don't need it anymore i think...
    // if (node.isRootFrame) {
    //     offset = formatVector(node.position);
    //     linkRotation = quaternionToRPY(node.quaternion);
    // } else if (node.parentFrame.isRootFrame) {
    //     const quaternion = new THREE.Quaternion();
    //     rotation = quaternionToRPY(
    //         quaternion.multiplyQuaternions(
    //             node.parentFrame.quaternion,
    //             node.quaternion,
    //         ),
    //     );
    // }

    // Visual, Collision, and Inertial properties position are all based off their Parent Reference Frame
    // In URDF That is the Joint, in SDF that is the Link. We implemented an Offset feature to make SDF easier
    // but that means we need to add that offset to each link children in URDF
    const offset = new THREE.Vector3().copy(node.offset);

    let xml = "";

    // Start link
    xml += `  <link name="${linkName}">\n`;

    // add all the visual and collision tags needed
    xml += generateVisuals(node, offset, projectTitle);
    xml += generateCollisions(node, offset, projectTitle);

    // Add inertial element
    xml += generateInertials(node, offset);

    // Check for sensors and add Gazebo plugin if applicable
    if (node.sensor.type) {
        const sensorXML = generateSensorXML(node);
        xml += sensorXML;
    }

    // End link
    xml += "  </link>\n";
    return xml;
}
/**
 *
 * @param node
 * @param linkName
 * @param parentName
 * @returns The XML for a single Joint
 */
function generateJoint(
    node: Frame,
    linkName: string,
    parentName: string | null,
): string {
    // Add joint if there's a parent link
    let xml = "";
    if (parentName) {
        // node.getWorldPosition(node.position);
        const origin = node.position;
        const quaternion = node.quaternion;
        const jointType = node.jointType;
        const normalizedAxis = node.axis.normalize();
        const min = node.min;
        const max = node.max;

        xml += `  <joint name="${parentName}_to_${linkName}" type="${jointType}">\n`;
        xml += `    <parent link="${parentName}" />\n`;
        xml += `    <child link="${linkName}" />\n`;
        xml += `    <origin xyz="${formatVector(origin)}" rpy="${quaternionToRPY(quaternion)}" />\n`;
        if (node.jointType !== "fixed") {
            xml += `    <axis xyz="${formatVector(normalizedAxis)}"/>\n`;
            if (node.jointType !== "continuous") {
                xml += `    <limit effort="1000.0" lower="${min}" upper="${max}" velocity="0.5"/>`;
            }
        }
        xml += "  </joint>\n";
    }
    return xml;
}
/**
 *
 * @param node
 * @param offset
 * @param projectTitle
 * @returns XML for all the Visual properties within a single link tag
 */
function generateVisuals(
    node: Frame,
    offset: THREE.Vector3,
    projectTitle: string,
): string {
    let xml = "";
    const visuals = node.visuals;
    for (const visual of visuals) {
        const origin = offset.add(visual.position);
        const quaternion = visual.quaternion;
        const scale = visual.scale;

        xml += "    <visual>\n";
        xml += `      <origin xyz="${formatVector(origin)}" rpy="${quaternionToRPY(quaternion)}" />\n`;
        xml += "      <geometry>\n";
        const geometryType = visual.shape;
        if (geometryType === "cube") {
            xml += `        <box size="${formatVector(visual.scale)}" />\n`;
        } else if (geometryType === "sphere") {
            xml += `        <sphere radius="${scale.x / 2}" />\n`;
        } else if (geometryType === "cylinder") {
            xml += `        <cylinder radius="${scale.x / 2}" length="${scale.z}" />\n`;
        } else if (geometryType === "mesh") {
            xml += `        <mesh filename="package://${projectTitle}_description/meshes/${visual.stlfile}" scale="${scale.x} ${scale.x} ${scale.x}" />\n`;
        }
        xml += "      </geometry>\n";
        xml += `      <material name="${visual.frame.name}-material">\n`;
        xml += `        <color rgba="${visual.color.r} ${visual.color.g} ${visual.color.b} 1" />\n`;
        xml += "      </material>\n";
        xml += "    </visual>\n";
    }
    return xml;
}
/**
 *
 * @param node
 * @param offset
 * @param projectTitle
 * @returns XML for all the Collision properties within a single link tag
 */
function generateCollisions(
    node: Frame,
    offset: THREE.Vector3,
    projectTitle: string,
): string {
    let xml = "";
    const collisions = node.collisions;
    for (const collision of collisions) {
        const origin = offset.add(collision.position);
        const quaternion = collision.quaternion;
        const scale = collision.scale;

        xml += "    <collision>\n";
        xml += `      <origin xyz="${formatVector(origin)}" rpy="${quaternionToRPY(quaternion)}" />\n`;
        xml += "      <geometry>\n";
        const geometryType = collision.shape;
        if (geometryType === "cube") {
            xml += `        <box size="${formatVector(scale)}" />\n`;
        } else if (geometryType === "sphere") {
            xml += `        <sphere radius="${scale.x / 2}" />\n`;
        } else if (geometryType === "cylinder") {
            xml += `        <cylinder radius="${scale.x / 2}" length="${scale.z}" />\n`;
        } else if (geometryType === "mesh") {
            xml += `        <mesh filename="package://${projectTitle}_description/meshes/${collision.stlfile}" scale="${scale.x} ${scale.x} ${scale.x}" />\n`;
        }
        xml += "      </geometry>\n";
        xml += "    </collision>\n";
    }
    return xml;
}
/**
 *
 * @param node
 * @param offset
 * @returns The inertia property for a single link tag
 */
function generateInertials(node: Frame, offset: THREE.Vector3): string {
    let xml = "";
    const inertia = node.inertia;
    const position = offset.add(inertia.position);
    const quaternion = inertia.quaternion;
    const mass = inertia.mass || 0;
    const { ixx, ixy, ixz, iyy, iyz, izz } = inertia;
    xml += "    <inertial>\n";
    xml += `      <origin xyz="${formatVector(position)}" rpy="${quaternionToRPY(quaternion)}" />\n`;
    xml += `      <mass value="${mass}" />\n`;
    xml += `      <inertia ixx="${ixx || 0}" ixy="${ixy || 0}" ixz="${ixz || 0}" iyy="${iyy || 0}" iyz="${iyz || 0}" izz="${izz || 0}" />\n`;
    xml += "    </inertial>\n";
    return xml;
}
