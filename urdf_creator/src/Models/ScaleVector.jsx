import * as THREE from "three";

export default class ScaleVector extends THREE.Vector3 {
    constructor(shape, x, y, z) {
        super(x, y, z);

        this.shape = shape;
    }

    set(x, y, z) {
        console.log("scale", x, y, z);
        switch (this.shape) {
            case "cube":
                super.set(x, y, z);
                return this;
            case "sphere":
                console.log(this.x, this.y, this.z, x, y, z);
                if (x !== this.getComponent(0)) {
                    super.set(x, x, x);
                } else if (y !== this.getComponent(1)) {
                    super.set(y, y, y);
                } else if (z !== this.getComponent(2)) {
                    super.set(new THREE.Vector3(z, z, z));
                }
                return this;
            case "cylinder":
                if (x !== this.getComponent(0)) {
                    super.set(x, x, z);
                } else if (y !== this.getComponent(1)) {
                    super.set(y, y, z);
                } else if (z !== this.getComponent(2)) {
                    super.set(x, y, z);
                }
                return this;
            default:
                console.log(this.shape);
                throw Error("shape provided to scale vector is not supported");
        }
    }

    multiply(vector) {
        // The multiply vector is provided as a <1,1,1>. If a axis has been scaled then only that component will be different. i.e. scale X then the multiply vector will be <x,1,1>
        switch (this.shape) {
            case "cube":
                super.multiply(vector);
                return this;
            case "sphere":
                const x = vector.getComponent(0);
                const y = vector.getComponent(1);
                const z = vector.getComponent(2);

                if (x !== this.getComponent(0) && x !== 1) {
                    super.multiply(new THREE.Vector3(x, x, x));
                } else if (y !== this.getComponent(1) && x !== 1) {
                    super.multiply(new THREE.Vector3(y, y, y));
                } else if (z !== this.getComponent(2) && x !== 1) {
                    super.multiply(new THREE.Vector3(z, z, z));
                }
                return this;
            case "cylinder":
                const a = Math.abs(vector.getComponent(0));
                const b = Math.abs(vector.getComponent(1));
                const c = vector.getComponent(2);

                if (a !== this.getComponent(0)) {
                    super.multiply(new THREE.Vector3(a, a, c));
                } else if (b !== this.getComponent(1)) {
                    super.multiply(new THREE.Vector3(b, b, c));
                } else if (c !== this.getComponent(2)) {
                    super.multiply(vector);
                }
                return this;
            default:
                console.log(this.shape);
                throw Error("shape provided to scale vector is not supported");
        }
    }
}
