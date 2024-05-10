// URDFGenerator.js
import { Link } from '../components/URDFContext/LinkClass';

/**
 * Serializes the entire URDF tree starting from the root link into a URDF file format string.
 * @param {Link} root - The root link of the URDF tree.
 * @returns {string} The URDF file format string.
 */
export function serializeURDF(root) {
    // Function to convert a single link into its URDF representation
    function linkToURDF(link) {
        let urdf = `<link name="${link.value}">\n`;
        urdf += `  <visual>\n`;
        urdf += `    <geometry>\n`;
        urdf += `      <${link.geometry_type} size="${link.size.width} ${link.size.height} ${link.size.depth}" />\n`;
        urdf += `    </geometry>\n`;
        urdf += `    <origin xyz="${link.position.x} ${link.position.y} ${link.position.z}" rpy="${link.orientation.roll} ${link.orientation.pitch} ${link.orientation.yaw}" />\n`;
        urdf += `  </visual>\n`;
        urdf += `  <collision>\n`;
        urdf += `    <geometry>\n`;
        urdf += `      <${link.geometry_type} size="${link.size.width} ${link.size.height} ${link.size.depth}" />\n`;
        urdf += `    </geometry>\n`;
        urdf += `    <origin xyz="${link.position.x} ${link.position.y} ${link.position.z}" rpy="${link.orientation.roll} ${link.orientation.pitch} ${link.orientation.yaw}" />\n`;
        urdf += `  </collision>\n`;
        urdf += `  <inertial>\n`;
        urdf += `    <mass value="1.0" />\n`;
        urdf += `    <origin xyz="${link.position.x} ${link.position.y} ${link.position.z}" rpy="${link.orientation.roll} ${link.orientation.pitch} ${link.orientation.yaw}" />\n`;
        urdf += `    <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0" />\n`;
        urdf += `  </inertial>\n`;
        urdf += `</link>\n`;

        if (link.joint && link.joint.parentFrameId) {
            urdf += `<joint name="${link.joint.selfFrameId}" type="${link.joint.jointType}">\n`;
            urdf += `  <parent link="${link.joint.parentFrameId}"/>\n`;
            urdf += `  <child link="${link.value}"/>\n`;
            urdf += `  <origin xyz="${link.position.x} ${link.position.y} ${link.position.z}" rpy="${link.orientation.roll} ${link.orientation.pitch} ${link.orientation.yaw}" />\n`;
            urdf += `  <axis xyz="0 0 1"/>\n`;
            urdf += `  <limit effort="300" lower="-3.14" upper="3.14" velocity="0.5"/>\n`;
            urdf += `</joint>\n`;
        }

        return urdf;
    }

    // Recursive function to handle all links in the URDF tree
    function serializeLinkRecursive(currentLink) {
        let result = linkToURDF(currentLink);
        currentLink.children.forEach(child => {
            child.joint.parentFrameId = currentLink.value; // Set the parent frame ID for the joint
            result += serializeLinkRecursive(child);
        });
        return result;
    }

    // Start the serialization from the root
    return `<?xml version="1.0" ?>\n<robot name="URDF_Robot">\n${serializeLinkRecursive(root)}</robot>`;
}

