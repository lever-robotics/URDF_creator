import * as THREE from "three";
import Link from "./Link";
import Frame, { Frameish } from "./Frame";

export default class JointVisualizer extends THREE.Object3D {
    link?: Link
    value: number;
    frame: Frameish;

    constructor() {
        super();
        this.value = 0;
    }

    duplicate() {
        return new JointVisualizer();
    }
}
