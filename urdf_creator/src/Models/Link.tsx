import * as THREE from "three";
import Frame from "./Frame";

export default class Link extends THREE.Object3D {
    frame?: Frame | null;
    constructor(offset: [number, number, number] = [0, 0, 0]) {
        super();

        this.position.set(...offset); // The offset from the joint
    }

    duplicate() {
        const clone = new Link([this.position.x, this.position.y, this.position.z]);
        return clone;
    }
}
