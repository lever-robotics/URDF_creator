import * as THREE from "three";
import Frame from "./Frame";

export default class Axis extends THREE.Line {
    declare material: THREE.LineBasicMaterial;
    _axis: THREE.Vector3;
    isFrame: boolean;
    frame?: Frame | null;

    constructor(axisRotation: [number, number, number] = [0, 0, 0]) {
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
        this.rotation.set(...axisRotation);
        this.isFrame = false;
    }

    // Normalized Axis of the Joint
    get axis() {
        // this.quaternion.setFromEuler(this.rotation);
        // this._axis.set(0, 0, 1);
        return this._axis;
    }

    duplicate() {
        const rotation_tuple: [number, number, number] = [this.rotation.x, this.rotation.y, this.rotation.z];
        return new Axis(rotation_tuple);
    }
}
