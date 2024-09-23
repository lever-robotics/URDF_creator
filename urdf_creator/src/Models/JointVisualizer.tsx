import * as THREE from "three";
import Link from "./Link";
import Frame from "./Frame";

export default class JointVisualizer extends THREE.Object3D {
    link?: Link
    value: number;
    frame?: Frame | null;

    constructor() {
        super();
        this.value = 0;
    }

    duplicate() {
        return new JointVisualizer();
    }
}
