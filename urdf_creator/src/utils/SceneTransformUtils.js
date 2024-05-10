// SceneTransformUtils.js
import * as THREE from 'three';

function parseXml(xmlStr) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(xmlStr, "text/xml");
    return xmlDoc;
}

export function XMLtoScene(xml) {
    const xmlDoc = parseXml(xml);

    // Example: Traverse xmlDoc and build your scene
    const scene = new THREE.Scene();
    const links = xmlDoc.getElementsByTagName('link');

    for (let i = 0; i < links.length; i++) {
        const link = links[i];
        // Assuming a simple box geometry for each link
        const geometry = new THREE.BoxGeometry(1, 1, 1);
        const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const mesh = new THREE.Mesh(geometry, material);
        scene.add(mesh);
    }

    return scene;
}

// Helper function to conver ScenetoXML
export const ScenetoXML = (scene) => {
    let xml = `<robot name="GeneratedRobot">\n`;

    // Helper to format vector as a string
    const formatVector = (vec) => `${vec.x} ${vec.y} ${vec.z}`;

    // Helper to convert rotation to URDF-compatible roll-pitch-yaw (rpy)
    const quaternionToEuler = (quaternion) => {
        const euler = new THREE.Euler();
        euler.setFromQuaternion(quaternion, 'XYZ');
        return `${euler.x} ${euler.y} ${euler.z}`;
    };

    // Traverse the scene
    scene.traverse((object) => {
        if (object instanceof THREE.Mesh) {
            const geometryType = object.geometry.type;
            const position = formatVector(object.position);
            const rotation = quaternionToEuler(object.quaternion);
            const scale = formatVector(object.scale);

            // Start link
            xml += `  <link name="${object.name || 'link'}">\n`;
            xml += `    <visual>\n`;
            xml += `      <origin xyz="${position}" rpy="${rotation}" />\n`;

            // Geometry
            if (geometryType === 'BoxGeometry') {
                const size = formatVector(new THREE.Vector3().setFromMatrixScale(object.matrixWorld));
                xml += `      <geometry>\n`;
                xml += `        <box size="${size.x} ${size.y} ${size.z}" />\n`;
                xml += `      </geometry>\n`;
            } else if (geometryType === 'SphereGeometry') {
                const radius = object.geometry.parameters.radius;
                xml += `      <geometry>\n`;
                xml += `        <sphere radius="${radius}" />\n`;
                xml += `      </geometry>\n`;
            } else if (geometryType === 'CylinderGeometry') {
                const radius = object.geometry.parameters.radiusTop;  // Assume top and bottom are the same
                const height = object.geometry.parameters.height;
                xml += `      <geometry>\n`;
                xml += `        <cylinder radius="${radius}" length="${height}" />\n`;
                xml += `      </geometry>\n`;
            }

            // Material
            if (object.material && object.material.color) {
                const color = object.material.color;
                xml += `      <material name="${object.material.name || 'material'}">\n`;
                xml += `        <color rgba="${color.r} ${color.g} ${color.b} 1" />\n`;
                xml += `      </material>\n`;
            }

            // End visual and link
            xml += `    </visual>\n`;
            xml += `  </link>\n`;
        }
    });

    // Close robot tag
    xml += `</robot>`;

    return xml;
};