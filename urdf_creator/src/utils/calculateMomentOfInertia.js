import * as THREE from 'three';

// Function to calculate the moment of inertia for different geometries
const calculateMomentOfInertia = (node) => {
    const mass = node.userData.mass || 1;
    const { scale, geometry, quaternion } = node;
    const { type: geometryType } = geometry;
    const inertia = {
        Ixx: 0,
        Ixy: 0,
        Ixz: 0,
        Iyy: 0,
        Iyz: 0,
        Izz: 0
    };

    if (geometryType === 'BoxGeometry') {
        const width = scale.x;
        const height = scale.y;
        const depth = scale.z;
        inertia.Ixx = (1 / 12) * mass * (height ** 2 + depth ** 2);
        inertia.Iyy = (1 / 12) * mass * (width ** 2 + depth ** 2);
        inertia.Izz = (1 / 12) * mass * (width ** 2 + height ** 2);
    } else if (geometryType === 'SphereGeometry') {
        const radius = scale.x;
        inertia.Ixx = (2 / 5) * mass * radius ** 2;
        inertia.Iyy = inertia.Ixx;
        inertia.Izz = inertia.Ixx;
    } else if (geometryType === 'CylinderGeometry') {
        const radius = scale.x;
        const height = scale.y;
        inertia.Ixx = (1 / 12) * mass * (3 * radius ** 2 + height ** 2);
        inertia.Iyy = (1 / 2) * mass * radius ** 2;
        inertia.Izz = inertia.Ixx;
    }

    // Apply rotation to the inertia tensor
    const quaternionMatrix = new THREE.Matrix4().makeRotationFromQuaternion(quaternion);
    const rotationMatrix = new THREE.Matrix3().setFromMatrix4(quaternionMatrix);
    const inertiaMatrix = new THREE.Matrix3();
    inertiaMatrix.set(
        inertia.Ixx, inertia.Ixy, inertia.Ixz,
        inertia.Ixy, inertia.Iyy, inertia.Iyz,
        inertia.Ixz, inertia.Iyz, inertia.Izz
    );

    const rotatedInertiaMatrix = inertiaMatrix.clone().multiply(rotationMatrix).multiply(rotationMatrix.clone().transpose());

    inertia.Ixx = rotatedInertiaMatrix.elements[0];
    inertia.Ixy = rotatedInertiaMatrix.elements[1];
    inertia.Ixz = rotatedInertiaMatrix.elements[2];
    inertia.Iyy = rotatedInertiaMatrix.elements[4];
    inertia.Iyz = rotatedInertiaMatrix.elements[5];
    inertia.Izz = rotatedInertiaMatrix.elements[8];

    return inertia;
};

export default calculateMomentOfInertia;