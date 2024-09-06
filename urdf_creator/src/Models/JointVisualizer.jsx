import * as THREE from "three";

export default class JointVisualizer extends THREE.Object3D {
    constructor() {
        super();
        this.value = 0;
    }

    clone() {
        return new JointVisualizer(this.position, this.type, this.min, this.max, this.value);
    }
}
