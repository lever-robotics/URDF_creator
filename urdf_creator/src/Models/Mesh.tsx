import * as THREE from "three";
import ScaleVector from "./ScaleVector";
import Frame, { Frameish } from "./Frame";
import { Color, Vector3 } from "three";

export default class Mesh extends THREE.Mesh {
    private _scale: ScaleVector;
    shape: string;
    isShape: boolean;
    customRenderBehaviors: {};
    frame: Frameish;
    material: THREE.MeshPhongMaterial;
    constructor(shape = "cube", scale: Vector3 = new Vector3(1, 1, 1), color = Math.random() * 0xffffff) {
        super();

        this.scale.copy(scale);

        this._scale = new ScaleVector(shape, scale);

        this.shape = shape;
        this.isShape = true; // Used to be detectable by the mouse

        this.geometry = defineGeometry(this, shape);
        this.material = new THREE.MeshPhongMaterial();
        this.color = new Color(color);

        this.customRenderBehaviors = {};

        function defineGeometry(context: Mesh, shape: string) {
            switch (shape) {
                default:
                case "cube":
                    Object.defineProperty(context, "scale", {
                        get() {
                            return this._scale;
                        },
                        set(newVector) {
                            this._scale.set(...newVector);
                        },
                    });

                    return new THREE.BoxGeometry(1, 1, 1);
                case "sphere":
                    Object.defineProperty(context, "scale", {
                        get() {
                            return this._scale;
                        },
                        set(newVector) {
                            this._scale.set(...newVector);
                        },
                    });

                    return new THREE.SphereGeometry(0.5, 32, 32);
                case "cylinder":
                    Object.defineProperty(context, "scale", {
                        get() {
                            return this._scale;
                        },
                        set(newVector) {
                            this._scale.set(...newVector);
                        },
                    });
                    const cylinder = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                    cylinder.rotateX(Math.PI / 2);

                    return cylinder;
            }
        }
    }

    get color() {
        return this.material.color;
    }

    get scaleVector() {
        return new THREE.Vector3(this._scale.x, this._scale.y, this._scale.z);
    }

    set color(color) {
        this.material.color.copy(color);
    }

    duplicate(): this {
        const clone = new Mesh(this.shape, this.scale, this.color.getHex());
        clone.color.copy(this.color);
        // I added the as this. could potentially cause stupid errors
        return clone as this;
    }

    onAfterRender = () => {
        this.frame?.updateInertia();
    };
}
