import * as THREE from "three";
import { Euler } from "three";
import Frame, { type Frameish } from "./Frame";

export default class Axis extends THREE.Line {
    declare material: THREE.LineBasicMaterial;
    axis: THREE.Vector3;
    frame: Frameish;

    constructor(axisRotation: Euler = new Euler(0, 0, 0)) {
        const originPoint = new THREE.Vector3();
        const lineAxis = new THREE.Vector3(0, 0, 1);
        const length = 10;
        const startPoint = originPoint
            .clone()
            .sub(lineAxis.clone().multiplyScalar(length / 2));
        const endPoint = originPoint
            .clone()
            .add(lineAxis.clone().multiplyScalar(length / 2));
        const points = [];
        points.push(startPoint, endPoint);

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
        material.visible = false;

        super(geometry, material);

        this.axis = lineAxis;
        this.rotation.copy(axisRotation);
    }

    duplicate() {
        return new Axis(this.rotation);
    }
}
