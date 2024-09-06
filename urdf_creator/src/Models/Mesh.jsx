import * as THREE from "three";
import ScaleVector from "./ScaleVector";

export default class Mesh extends THREE.Mesh {
    constructor(shape = "cube", scale = [1, 1, 1], color = Math.random() * 0xffffff) {
        super();

        this.scale.set(...Object.values(scale));

        this._scale = new ScaleVector(shape, ...Object.values(scale));

        this.shape = shape;
        this.isShape = true; // Used to be detectable by the mouse

        this.geometry = defineGeometry(this, shape);
        this.material = new THREE.MeshPhongMaterial();
        this.color = color;

        this.customRenderBehaviors = {};

        // There was a problem with scaling spheres and cylinders based off render. It could cause a bug where the user would enter a radius value that they probably want to be exact and then the render would average that input out to uniformly scale the object. So a user would enter a diameter of 10 and then the sphere would have a diameter of 3.33. To fix this I customized the scaling behavior with a new Vector3 class called scale vector. It is probablly not the most elegant solution but it is functional.
        function defineGeometry(context, shape) {
            switch (shape) {
                case "cube":
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
                default:
                    return;
            }
        }
    }

    get color() {
        return this.material.color;
    }

    get scale() {
        return this._scale;
    }

    set color(color) {
        this.material.color.set(color);
    }

    clone() {
        const clone = new Mesh(this.shape, this.scale, this.color);
        clone.color.copy(this.color);
        return clone;
    }

    onAfterRender = () => {
        this.frame.updateInertia();
    };
}
