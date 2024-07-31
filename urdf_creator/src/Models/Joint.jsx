import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor(
        jointPosition = [0, 0, 0],
        axis = [0, 0, 1],
        type = "fixed",
        jointMin = -1,
        jointMax = 1
    ) {
        const originPoint = new THREE.Vector3(...jointPosition);
        const lineAxis = new THREE.Vector3(...axis);
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
        material.visible = type !== "fixed";

        super(geometry, material);

        this.position.set(...jointPosition);
        this.savedPosition = new THREE.Vector3(...jointPosition);
        this.savedRotation = new THREE.Euler().copy(this.rotation);

        this._min = jointMin;
        this._max = jointMax;
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

    get max() {
        return this._max;
    }

    set max(value) {
        this._max = value;
    }

    // Normalized Axis of the Joint
    get axis() {
        this._quaternion.setFromEuler(this.rotation);
        this._axis.set(0, 0, 1);
        return this._axis;
    }

    clone() {
        return new Joint(
            this.position,
            this.axis,
            this.type,
            this.min,
            this.max
        );
    }
}
