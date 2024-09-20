import * as THREE from "three";
import Frame from "./Frame";

export default class Inertia {
    frame?: Frame | null;
    mass: number;
    ixx: number;
    iyy: number;
    izz: number;
    ixy: number;
    ixz: number;
    iyz: number;
    customInertia: boolean;



    constructor(mass = 1, ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0) {
        this.customInertia = false;
        this.mass = mass;
        this.ixx = ixx;
        this.iyy = iyy;
        this.izz = izz;
        this.ixy = ixy;
        this.ixz = ixz;
        this.iyz = iyz;
    }

    // call every time the scaleing of the shape is changed
    updateInertia(threeObject: Frame) {
        if (this.customInertia) return;

        const shape = threeObject.shape;
        if (shape === 'cube') {
            const width = threeObject.link!.scale.x;
            const height = threeObject.link!.scale.y;
            const depth = threeObject.link!.scale.z;
            this.ixx = (1 / 12) * this.mass * (height ** 2 + depth ** 2);
            this.iyy = (1 / 12) * this.mass * (width ** 2 + depth ** 2);
            this.izz = (1 / 12) * this.mass * (width ** 2 + height ** 2);
        } else if (shape === 'cylinder') {
            const radius = threeObject.link!.scale.x;
            const height = threeObject.link!.scale.y;
            this.ixx = (1 / 12) * this.mass * (3 * radius ** 2 + height ** 2);
            this.iyy = (1 / 2) * this.mass * radius ** 2;
            this.izz = this.ixx;
        } else if (shape === 'sphere') {
            const radius = threeObject.link!.scale.x;
            this.ixx = (2 / 5) * this.mass * radius ** 2;
            this.iyy = this.ixx;
            this.izz = this.ixx;
        } else {
            console.error('Invalid shape input');
        }
    }

    setCustomInertia(type: string, inertia: number) {
        this.customInertia = true;
        switch (type) {
            case 'ixx':
            this.ixx = inertia;
            break;
            case 'iyy':
            this.iyy = inertia;
            break;
            case 'izz':
            this.izz = inertia;
            break;
            case 'ixy':
            this.ixy = inertia;
            break;
            case 'ixz':
            this.ixz = inertia;
            break;
            case 'iyz':
            this.iyz = inertia;
            break;
            default:
            console.error('Invalid inertia type');
        }
    }

    updateMass(newMass: number, object: Frame) {
        this.mass = newMass;
        if (!this.customInertia) {
            this.updateInertia(object);
        }
    }

    duplicate() {
        const clone = new Inertia(this.mass, this.ixx, this.iyy, this.izz, this.ixy, this.ixz, this.iyz);
        clone.customInertia = this.customInertia;
        return clone;
    }
}