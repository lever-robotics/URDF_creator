//XMLtoScene convert a XML file into a display for Three

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