import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor(origin, ax, type, jointMin, jointMax) {

        const point = new THREE.Vector3(...origin);
        const axis = new THREE.Vector3(...ax);
        const length = 10;
        const startPoint = point
            .clone()
            .sub(axis.clone().multiplyScalar(length / 2));
        const endPoint = point
            .clone()
            .add(axis.clone().multiplyScalar(length / 2));
        const points = [];
        points.push(startPoint);
        points.push(endPoint);

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
        material.visible = type !== "fixed";

        super(geometry, material);

        this.min = jointMin ?? -1;
        this.max = jointMax ?? 1;
        this.type = type;
        this.origin = origin;
        this.axis = axis;
    }

}
