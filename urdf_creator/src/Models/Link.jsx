import * as THREE from "three";

export default class Link extends THREE.Object3D {
    constructor(offset = [0, 0, 0]) {
        super();

        this.position.set(...offset); // The offset from the joint
    }

    clone() {
        const clone = new Link(this.position);
        return clone;
    }
}
