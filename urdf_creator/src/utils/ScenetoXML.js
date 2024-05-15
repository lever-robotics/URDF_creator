import * as THREE from 'three';

// Helper function to convert Scene to URDF-compatible XML
export const ScenetoXML = (scene) => {
    console.log("Converting Scene to XML");
    console.log(scene);

    let xml = `<robot name="GeneratedRobot">\n`;

    // Helper to format vector as a string
    const formatVector = (vec) => `${vec.x} ${vec.y} ${vec.z}`;

    // Helper to convert rotation to URDF-compatible roll-pitch-yaw (rpy)
    const quaternionToEuler = (quaternion) => {
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quaternion, 'XYZ');
        return `${euler.x} ${euler.y} ${euler.z}`;
    };

    // Function to process a single node
    const processNode = (node) => {
        if (node instanceof THREE.Mesh) {
            const geometryType = node.geometry.type;
            const position = formatVector(node.position);
            const rotation = quaternionToEuler(node.quaternion);
            const scale = formatVector(node.scale);

            // Start link
            xml += `  <link name="${node.name || 'link'}">\n`;
            xml += `    <visual>\n`;
            xml += `      <origin xyz="${position}" rpy="${rotation}" />\n`;

            // Geometry
            if (geometryType === 'BoxGeometry') {
                const size = formatVector(new THREE.Vector3().setFromMatrixScale(node.matrixWorld));
                xml += `      <geometry>\n`;
                xml += `        <box size="${size.x} ${size.y} ${size.z}" />\n`;
                xml += `      </geometry>\n`;
            } else if (geometryType === 'SphereGeometry') {
                const radius = node.geometry.parameters.radius;
                xml += `      <geometry>\n`;
                xml += `        <sphere radius="${radius}" />\n`;
                xml += `      </geometry>\n`;
            } else if (geometryType === 'CylinderGeometry') {
                const radius = node.geometry.parameters.radiusTop;  // Assume top and bottom are the same
                const height = node.geometry.parameters.height;
                xml += `      <geometry>\n`;
                xml += `        <cylinder radius="${radius}" length="${height}" />\n`;
                xml += `      </geometry>\n`;
            }

            // Material
            if (node.material && node.material.color) {
                const color = node.material.color;
                xml += `      <material name="${node.material.name || 'material'}">\n`;
                xml += `        <color rgba="${color.r} ${color.g} ${color.b} 1" />\n`;
                xml += `      </material>\n`;
            }

            // End visual and link
            xml += `    </visual>\n`;
            xml += `  </link>\n`;
        }

        // Recursively process children
        node.children.forEach(child => processNode(child));
    };

    // Find the base node and start processing
    const baseNode = scene.children.find(child => child.type === 'Mesh');
    if (baseNode) {
        processNode(baseNode);
    }

    // Close robot tag
    xml += `</robot>`;

    return xml;
};
