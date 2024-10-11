import * as THREE from "three";
import Frame, { type Frameish } from "./Frame";
import type Link from "./Link";

export default class JointVisualizer extends THREE.Object3D {
    link?: Link;
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
