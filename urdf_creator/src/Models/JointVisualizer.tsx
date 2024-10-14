import * as THREE from "three";
import Frame, { type Frameish } from "./Frame";
import type Link from "./Link";

export default class JointVisualizer extends THREE.Object3D {
    link?: Link;
    frame: Frameish;

    duplicate() {
        return new JointVisualizer();
    }
}
