import * as THREE from "three";
import Frame from "../Models/Frame";
import type ThreeScene from "../components/ThreeDisplay/ThreeScene";
import findRootFrame from "./findRootFrame";
import { generateSensorSDF } from "./generateSensorSDF";
import { quaternionToRPY } from "./quaternionToRPY";

// Helper function to convert Scene to SDF-compatible XML
export const ScenetoSDF = (scene: ThreeScene, projectTitle: string) => {
    let xml = `<sdf version="1.6">\n`;
    let pub_joint_states = `    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">\n`;
    // plugin for publishing joint states from gazebo
    pub_joint_states += "        <ros>\n";
    pub_joint_states +=
        "            <remapping>~/out:=/joint_states</remapping>\n";
    pub_joint_states += "        </ros>\n";
    pub_joint_states += "        <update_rate>5</update_rate>\n";
    if (scene === undefined) {
        xml += "</sdf>";
        return xml;
    }
    if (!scene.rootFrame) {
        xml += "</sdf>";
        return xml;
    }

    xml += `<model name="${projectTitle.replace(" ", "_")}" canonical_link='base_link'>\n`;
    //put on static for debugging
    xml += "  <static>false</static>\n";
    xml += `  <pose relative_to='world'>0 0 0 0 0 0</pose>\n`;

    // Helper to format vector as a string and flip y and z coordinates
    const formatVector = (vec: THREE.Vector3) => `${vec.x} ${vec.y} ${vec.z}`;

    // Variables to keep track of link naming
    let linkIndex = 0;
    const generateLinkName = (node: Frame) => {
        return (
            node.name || (linkIndex === 0 ? "base_link" : `link${linkIndex}`)
        );
    };

    // Function to process a single node
    const processNode = (node: Frame, parentName: string | null = null) => {
        if (node instanceof Frame) {
            const linkName = generateLinkName(node);
            linkIndex += 1;

            //offset from parent to joint origin
            let offset = formatVector(node.link.position.clone().negate()); // offset from parent link to joint origin as sdf is link defined and not joint defined
            // position of link in relation to parent
            const position = formatVector(
                node.position.clone().add(node.link.position),
            );
            const rotation = quaternionToRPY(node.quaternion);
            let linkRotation = "0 0 0";

            if (node.isRootFrame) {
                offset = formatVector(node.position);
                linkRotation = quaternionToRPY(node.quaternion);
            }

            // Start link
            xml += `  <link name="${linkName}">\n`;
            if (node.isRootFrame) {
                xml += `    <pose relative_to='__model__'>${formatVector(node.position)} ${quaternionToRPY(node.quaternion)}</pose>\n`;
            } else {
                xml += `    <pose relative_to='${parentName}'>${position} ${rotation}</pose>\n`;
            }

            // add all the visual and collision tags needed
            const visuals = node.visuals;
            const collisions = node.collisions;

            if (visuals) {
                for (const visual of visuals) {
                    const color = visual.material.color;
                    const materialXML = `      <material>\n         <ambient>${color.r} ${color.g} ${color.b} 1</ambient>\n         <diffuse>${color.r} ${color.g} ${
                        color.b
                    } 1</diffuse>\n         <specular>0.1 0.1 0.1 1</specular>\n         <emissive>0 0 0 1</emissive>\n       </material>\n`;
                    xml += `    <visual name='${node.name} visual'>\n`;
                    xml += `      <pose>${formatVector(visual.position)} ${quaternionToRPY(visual.quaternion)}</pose>\n`;
                    xml += "      <geometry>\n";
                    const geometryType = visual.geometry.type;
                    if (geometryType === "BoxGeometry") {
                        xml += `        <box>\n          <size>${formatVector(visual.scale)}</size>\n        </box>\n`;
                    } else if (geometryType === "SphereGeometry") {
                        xml += `        <sphere>\n          <radius>${visual.scale.x / 2}</radius>\n        </sphere>\n`;
                    } else if (geometryType === "CylinderGeometry") {
                        xml += `        <cylinder>\n          <radius>${visual.scale.x / 2}</radius>\n          <length>${visual.scale.z}</length>\n        </cylinder>\n`;
                    } else if (visual.shape === "mesh") {
                        xml += `        <mesh>\n          <uri>model://meshes/${visual.stlfile}</uri>\n          <scale>${formatVector(visual.scale)}</scale>\n        </mesh>\n`;
                    }
                    xml += "      </geometry>\n";
                    xml += materialXML;
                    xml += "    </visual>\n";
                }
            }
            if (collisions) {
                for (const collision of collisions) {
                    xml += `    <collision name='${node.name} collision'>\n`;
                    xml += `      <pose>${formatVector(collision.position)} ${quaternionToRPY(collision.quaternion)}</pose>\n`;
                    xml += "      <geometry>\n";
                    const geometryType = collision.geometry.type;
                    if (geometryType === "BoxGeometry") {
                        xml += `        <box>\n          <size>${formatVector(collision.scale)}</size>\n        </box>\n`;
                    } else if (geometryType === "SphereGeometry") {
                        xml += `        <sphere>\n          <radius>${collision.scale.x / 2}</radius>\n        </sphere>\n`;
                    } else if (geometryType === "CylinderGeometry") {
                        xml += `        <cylinder>\n          <radius>${collision.scale.x / 2}</radius>\n          <length>${collision.scale.z}</length>\n        </cylinder>\n`;
                    } else if (collision.shape === "mesh") {
                        xml += `        <mesh>\n          <uri>model://meshes/${collision.stlfile}</uri>\n          <scale>${formatVector(collision.scale)}</scale>\n        </mesh>\n`;
                    }
                    xml += "      </geometry>\n";
                    xml += "    </collision>\n";
                }
            }

            // Add inertial element
            const mass = node.inertia.mass || 0;
            const { ixx, ixy, ixz, iyy, iyz, izz } = node.inertia;
            xml += `    <inertial>\n      <mass>${mass}</mass>\n      <inertia>\n        <ixx>${ixx || 0}</ixx>\n        <ixy>${ixy || 0}</ixy>\n        <ixz>${
                ixz || 0
            }</ixz>\n        <iyy>${iyy || 0}</iyy>\n        <iyz>${iyz || 0}</iyz>\n        <izz>${izz || 0}</izz>\n      </inertia>\n    </inertial>\n`;

            // Check for sensors and add Gazebo plugin if applicable
            if (node.sensor.type) {
                const sensorSDF = generateSensorSDF(node);
                xml += sensorSDF;
            }

            // End link
            xml += "  </link>\n";

            // Add joint if there's a parent link
            if (parentName) {
                let jointType = node.jointType;
                if (jointType === "continuous") {
                    jointType = "revolute";
                }
                // if (geometryType === "SphereGeometry") {
                //     jointType = "ball";
                // }

                xml += `  <joint name="${parentName}_to_${linkName}" type="${jointType}">\n`;

                // because urdf is dumb, children links are connected to parent joints, not parent meshes
                // this code accounts for that and sets the joint origin in relation to the parent's joint origin
                // ie it add the links position to its own since it isnt passed with
                const originInRelationToParentsJoint = new THREE.Vector3();
                originInRelationToParentsJoint.copy(node.position);
                originInRelationToParentsJoint.add(node.parentFrame.position);

                if (node.parentFrame.isRootFrame) {
                    node.getWorldPosition(originInRelationToParentsJoint);
                }

                xml += `    <pose relative_to='${linkName}'>${offset} 0 0 0 </pose>\n`;

                xml += `    <parent>${parentName}</parent>\n`;
                xml += `    <child>${linkName}</child>\n`;
                if (node.jointType !== "fixed") {
                    const quaternion = new THREE.Quaternion();
                    quaternion.setFromEuler(node.jointVisualizer.rotation);
                    const newAxis = new THREE.Vector3(
                        ...node.axis.axis,
                    ).applyQuaternion(quaternion);
                    xml += "    <axis>\n";
                    xml += `        <xyz expressed_in='${linkName}'>${formatVector(newAxis)}</xyz>\n`;
                    if (node.jointType === "revolute") {
                        xml += "        <limit>";
                        xml += `            <lower>${node.min}</lower>`;
                        xml += `            <upper>${node.max}</upper>`;
                        xml += "        </limit>\n";
                    }
                    xml += "    </axis>\n";
                }
                xml += "  </joint>\n";
            }

            // create code to add the joint states that should be published by Gazebo
            if (node.jointType !== "fixed")
                pub_joint_states += `       <joint_name>${parentName}_to_${linkName}</joint_name>\n`;

            // Recursively process children with the correct parent name
            for (const child of node.getFrameChildren()) {
                processNode(child, linkName);
            }
        }
    };

    // Find the base node and start processing
    const rootFrame = scene.rootFrame;
    if (rootFrame) processNode(rootFrame);

    pub_joint_states += "    </plugin>\n";
    xml += pub_joint_states;

    // Close model and sdf tags
    xml += "</model>\n";
    xml += "</sdf>";

    return xml;
};
