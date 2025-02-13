import * as THREE from "three";
import type { Vector3 } from "three";

export default class ScaleVector extends THREE.Vector3 {
    shape: string;

    constructor(shape: string, scale: Vector3) {
        const { x, y, z } = scale;
        super(x, y, z);

        this.shape = shape;
    }

    // set(x: number, y: number, z: number) {
    //     switch (this.shape) {
    //         case "cube":
    //             super.set(x, y, z);
    //             return this;
    //         case "sphere":
    //         case "mesh":
    //             if (x !== this.getComponent(0)) {
    //                 super.set(x, x, x);
    //             } else if (y !== this.getComponent(1)) {
    //                 super.set(y, y, y);
    //             } else if (z !== this.getComponent(2)) {
    //                 super.set(z, z, z);
    //             }
    //             return this;
    //         case "cylinder":
    //             if (x !== this.getComponent(0)) {
    //                 super.set(x, x, z);
    //             } else if (y !== this.getComponent(1)) {
    //                 super.set(y, y, z);
    //             } else if (z !== this.getComponent(2)) {
    //                 super.set(x, y, z);
    //             }
    //             return this;
    //         default:
    //             throw Error("shape provided to scale vector is not supported");
    //     }
    // }

    // copy(scale: ScaleVector) {
    //     const x = scale.x;
    //     const y = scale.y;
    //     const z = scale.z;

    //     switch (this.shape) {
    //         case "cube":
    //             super.set(x, y, z);
    //             return this;
    //         case "sphere": {
    //             const averageSphere = (x + y + z) / 3;
    //             super.set(averageSphere, averageSphere, averageSphere);
    //             return this;
    //         }
    //         case "cylinder": {
    //             const averageCylinder = (x + y) / 2;
    //             super.set(averageCylinder, averageCylinder, z);
    //             return this;
    //         }
    //         default:
    //             throw Error("shape provided to scale vector is not supported");
    //     }
    // }

    // THREE.Vector3 has a function called multiply that is called if the vector 3 is a scene object's scale whenever the object is scaled by transform controls
    // this function hijacks the normal scale function and ensures that spheres and cylinders scale uniformly
    multiply(vector: THREE.Vector3) {
        // The multiply vector is provided as a <1,1,1>. If a axis has been scaled then only that component will be different. i.e. scale X then the multiply vector will be <x,1,1>
        switch (this.shape) {
            case "cube":
                super.multiply(vector);
                break;
            case "mesh":
            case "sphere": {
                const x = vector.x;
                const y = vector.y;
                const z = vector.z;

                if (vector.x !== 1) {
                    super.multiply(new THREE.Vector3(x, x, x));
                } else if (vector.y !== 1) {
                    super.multiply(new THREE.Vector3(y, y, y));
                } else if (vector.z !== 1) {
                    super.multiply(new THREE.Vector3(z, z, z));
                }
                break;
            }
            case "cylinder": {
                const x = vector.x;
                const y = vector.y;
                const z = vector.z;

                if (vector.x !== 1) {
                    super.multiply(new THREE.Vector3(x, x, z));
                } else if (vector.y !== 1) {
                    super.multiply(new THREE.Vector3(y, y, z));
                } else if (vector.z !== 1) {
                    super.multiply(vector);
                }
                break;
            }
            default:
                throw Error("shape provided to scale vector is not supported");
        }

        if (this.x < 0) {
            this.x = 0.001;
        }
        if (this.y < 0) {
            this.y = 0.001;
        }
        if (this.z < 0) {
            this.z = 0.001;
        }
        return this;
    }
}
