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

        this.position.set(...origin);
        this.savedPosition = new THREE.Vector3(...origin);
        this.savedRotation = new THREE.Euler().copy(this.rotation);

        this._min = jointMin ?? -1;
        this._max = jointMax ?? 1;
        this._type = type;
        this._axis = new THREE.Vector3(0, 0, 1);
        this._quaternion = new THREE.Quaternion();
    }

    get type() {
        return this._type;
    }

    set type(type) {
        this._type = type;

        switch (type) {
            case "fixed":
                this.material.visible = false;
                break;
            case "prismatic":
                this._min = -1;
                this._max = 1;
                this.material.visible = true;
                break;
            case "revolute":
            case "continuous":
                this._min = -3.14;
                this._max = 3.14;
                this.material.visible = true;
                break;
        }
    }

    get min() {
        return this._min;
    }

    set min(value) {
        this._min = value;
    }

    set max(value) {
        this._max = value;
    }

    get max() {
        return this._max;
    }

    get axis() {
        this._quaternion.setFromEuler(this.rotation);
        this._axis.set(0, 0, 1);
        return this._axis;
    }
}
