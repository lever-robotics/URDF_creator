import * as THREE from "three";

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene) => {
    //console.log("Converting Scene to XML");
    //console.log(scene);

    let xml = `<robot name="GeneratedRobot">\n`;

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
    const generateLinkName = () => (linkIndex === 0 ? "base_link" : `link${linkIndex}`);

    // Function to process a single node
    const processNode = (node, parentName = null) => {
        if (node instanceof THREE.Mesh) {
            const linkName = generateLinkName();
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
                const size = formatVector(new THREE.Vector3().setFromMatrixScale(node.matrixWorld));
                geometryXML = `      <geometry>\n        <box size="${size}" />\n      </geometry>\n`;
            } else if (geometryType === "SphereGeometry") {
                const radius = node.geometry.parameters.radius;
                geometryXML = `      <geometry>\n        <sphere radius="${radius}" />\n      </geometry>\n`;
            } else if (geometryType === "CylinderGeometry") {
                const radius = node.geometry.parameters.radiusTop; // Assume top and bottom are the same
                const height = node.geometry.parameters.height;
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
            xml += `    <inertial>\n`;
            xml += `      <origin xyz="0 0 0" rpy="0 0 0" />\n`;
            xml += `      <mass value="1.0" />\n`; // Assuming a default mass of 1 kg
            xml += `      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />\n`; // Simplified inertia
            xml += `    </inertial>\n`;

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
            node.userData.scaler.children.forEach((child) => processNode(child, linkName));
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
