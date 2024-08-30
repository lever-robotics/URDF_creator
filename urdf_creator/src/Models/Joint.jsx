import * as THREE from "three";

export default class Joint extends THREE.Object3D {
    constructor(jointPosition = [0, 0, 0], type = "fixed", jointMin = -1, jointMax = 1, value = 0) {
        super();

        this.position.set(...jointPosition);

        this._min = jointMin;
        this._max = jointMax;
        this._type = type;
        this._axis = new THREE.Vector3(0, 0, 1);
        this._quaternion = new THREE.Quaternion();
        this.value = value;
    }

    get type() {
        return this._type;
    }

    set type(type) {
        this._type = type;

        switch (type) {
            case "fixed":
                break;
            case "prismatic":
                this._min = -1;
                this._max = 1;
                break;
            case "revolute":
            case "continuous":
                this._min = -3.14;
                this._max = 3.14;
                break;
            default:
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

    clone() {
        return new Joint(this.position, this.type, this.min, this.max, this.value);
    }
}
