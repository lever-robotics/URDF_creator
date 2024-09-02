import * as THREE from "three";

export default class Axis extends THREE.Line {
    constructor() {
        const originPoint = new THREE.Vector3();
        const lineAxis = new THREE.Vector3(0, 0, 1);
        const length = 10;
        const startPoint = originPoint.clone().sub(lineAxis.clone().multiplyScalar(length / 2));
        const endPoint = originPoint.clone().add(lineAxis.clone().multiplyScalar(length / 2));
        const points = [];
        points.push(startPoint, endPoint);

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00ffff });

        super(geometry, material);

        this._axis = lineAxis;
    }

    get type() {
        return this._type;
    }

    set type(value) {
        this._type = value;

        switch (value) {
            case "fixed":
                this.material.visible = false;
                break;
            case "prismatic":
            case "revolute":
            case "continuous":
                this.material.visible = true;
                break;
            default:
                break;
        }
    }

    // Normalized Axis of the Joint
    get axis() {
        // this.quaternion.setFromEuler(this.rotation);
        // this._axis.set(0, 0, 1);
        return this._axis;
    }

    clone() {
        const newAxis = new Axis();
        newAxis.position.copy(this.position);
        newAxis.quaternion.copy(this.quaternion);
        return newAxis;
    }
}
