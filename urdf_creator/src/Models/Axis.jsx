import * as THREE from "three";

export default class Axis extends THREE.Line {
    constructor(axisRotation = [0,0,0,0]) {
        const originPoint = new THREE.Vector3();
        const lineAxis = new THREE.Vector3(0, 0, 1);
        const length = 10;
        const startPoint = originPoint.clone().sub(lineAxis.clone().multiplyScalar(length / 2));
        const endPoint = originPoint.clone().add(lineAxis.clone().multiplyScalar(length / 2));
        const points = [];
        points.push(startPoint, endPoint);

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
        material.visible = false;

        super(geometry, material);

        this._axis = lineAxis;
        this.rotation.set(...Object.values(axisRotation).slice(1,4));
        this.isFrame = false;
    }

    // Normalized Axis of the Joint
    get axis() {
        // this.quaternion.setFromEuler(this.rotation);
        // this._axis.set(0, 0, 1);
        return this._axis;
    }

    clone() {
        return new Axis(this.rotation);
    }
}
