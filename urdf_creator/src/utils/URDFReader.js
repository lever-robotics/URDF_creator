// URDFReader.js
import { Link } from '../components/URDFContext/LinkClass';

/**
 * Parses a URDF XML string and converts it into a tree of Link objects.
 * @param {string} urdfText The URDF XML text.
 * @returns {Link | null} The root node of the URDF tree, or null if parsing fails.
 */
export const deserializeURDF = (urdfText) => {
    // Initialize the DOM parser
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(urdfText, 'text/xml');

    // Check for errors in parsing
    const parseError = xmlDoc.getElementsByTagName('parsererror');
    if (parseError.length) {
        console.error('Error parsing XML:', parseError[0].textContent);
        return null;
    }

    // Helper function to extract vector attributes like position
    const parseVector = (text) => {
        return text.split(' ').map(Number);
    };

    // Helper function to extract position and orientation
    const extractPositionOrientation = (element) => {
        const position = { x: 0, y: 0, z: 0 };
        const orientation = { roll: 0, pitch: 0, yaw: 0 };

        const originElement = element.querySelector('origin');
        if (originElement) {
            const xyz = originElement.getAttribute('xyz');
            if (xyz) {
                const [x, y, z] = parseVector(xyz);
                position.x = x;
                position.y = y;
                position.z = z;
            }
            const rpy = originElement.getAttribute('rpy');
            if (rpy) {
                const [roll, pitch, yaw] = parseVector(rpy);
                orientation.roll = roll;
                orientation.pitch = pitch;
                orientation.yaw = yaw;
            }
        }
        return { position, orientation };
    };

    // Construct the Link objects from link elements
    const linkElements = xmlDoc.getElementsByTagName('link');
    const links = {};
    for (let i = 0; i < linkElements.length; i++) {
        const linkEl = linkElements[i];
        const name = linkEl.getAttribute('name');
        if (!name) continue;

        const { position, orientation } = extractPositionOrientation(linkEl);

        links[name] = new Link(name, { position, orientation });
    }

    // Process the joints and build the tree structure
    const jointElements = xmlDoc.getElementsByTagName('joint');
    for (let i = 0; i < jointElements.length; i++) {
        const jointEl = jointElements[i];
        const name = jointEl.getAttribute('name');
        const type = jointEl.getAttribute('type');

        const parentLinkName = jointEl.querySelector('parent')?.getAttribute('link');
        const childLinkName = jointEl.querySelector('child')?.getAttribute('link');

        if (parentLinkName && childLinkName && links[parentLinkName] && links[childLinkName]) {
            const { position, orientation } = extractPositionOrientation(jointEl);

            // Update the joint info in the child link
            links[childLinkName].joint = {
                parentFrameId: parentLinkName,
                selfFrameId: childLinkName,
                jointType: type,
                ...position,
                ...orientation
            };

            // Set the parent-child relationship
            links[parentLinkName].children.push(links[childLinkName]);
        }
    }

    // Find the root link (no parent in the joint hierarchy)
    const rootLink = Object.values(links).find(link =>
        !Object.values(links).some(l => l.children.includes(link))
    );

    return rootLink || null;
};

