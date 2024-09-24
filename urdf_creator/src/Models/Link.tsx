import * as THREE from "three";
import Frame, { Frameish } from "./Frame";
import { Vector3 } from "three";

export default class Link extends THREE.Object3D {
    frame: Frameish;
    constructor(offset: Vector3 = new Vector3(0, 0, 0)) {
        super();

        this.position.copy(offset); // The offset from the joint
    }

    duplicate() {
        const clone = new Link(this.position);
        return clone;
    }
}
