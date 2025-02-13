import * as THREE from "three";
import { Vector3 } from "three";
import Frame, { type Frameish } from "./Frame";

export default class Link extends THREE.Object3D {
    frame: Frameish;
    shape: string;
    constructor(offset: Vector3 = new Vector3(0, 0, 0), shape = "cube") {
        super();
        this.shape = shape;
        this.position.copy(offset); // The offset from the joint
    }

    duplicate() {
        const clone = new Link(this.position, this.shape);
        return clone;
    }
}
