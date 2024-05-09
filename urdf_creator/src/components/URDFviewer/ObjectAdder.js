import React, { useContext, useEffect } from 'react';
import * as THREE from 'three';
import { URDFContext } from '../URDFContext/URDFContext';

const ObjectAdder = ({ selectedShape, scene }) => {
    const { setSelectedPart } = useContext(URDFContext);

    useEffect(() => {
        function addShape(position) {
            let geometry;
            switch (selectedShape) {
                case "sphere":
                    geometry = new THREE.SphereGeometry(0.5, 32, 32);
                    break;
                case "cylinder":
                    geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                    break;
                default: // "cube"
                    geometry = new THREE.BoxGeometry(1, 1, 1);
            }
            const material = new THREE.MeshBasicMaterial({ color: Math.random() * 0xffffff });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.position.copy(position).add(new THREE.Vector3(0, 0.5, 0));
            scene.add(mesh);
            setSelectedPart(mesh); // Optionally select the part after adding
        }

        addShape(new THREE.Vector3(0, 0, 0)); // Add a shape at the origin for demo purposes
    }, [selectedShape, scene, setSelectedPart]);

    return null;
};

export default ObjectAdder;
