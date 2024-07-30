import * as THREE from "three";
import ScaleVector from "./ScaleVector";

export default class Link extends THREE.Mesh {
    constructor(offset, shape, scale) {
        super();

        this.position.set(...offset); // The offset from the joint


        this.scale.set(...scale);
        this._scale = new ScaleVector(shape, ...scale);
        this.shape = shape;

        this.isShape = true;

        this.geometry = defineGeometry(this, shape);
        this.material = new THREE.MeshPhongMaterial();
        this.color = Math.random() * 0xffffff;

        this.customRenderBehaviors = {
            defineRenderBehavior: defineRenderBehavior(shape),
        };

        //***Helper Function***/
        function defineGeometry(context, shape, a, b, c) {
            switch (shape) {
                case "cube":
                    // a = width
                    // b = height
                    // c = depth
                    return new THREE.BoxGeometry(1, 1, 1);
                case "sphere":
                    // a = radius
                    // b = widthSegments
                    // c = heightSegments
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
                    // a = top and bottom radius
                    // b = height
                    // c = radialSegments = 'number of segmented faces around circumference
                    Object.defineProperty(context, "scale", {
                        get() {
                            return this._scale;
                        },
                        set(newVector) {
                            this._scale.set(...newVector);
                        },
                    });
                    const cylinder = new THREE.CylinderGeometry(
                        0.5,
                        0.5,
                        1,
                        32
                    );
                    cylinder.rotateX(Math.PI / 2);
                    return cylinder;
                default:
                    return;
            }
        }

        // There was a problem with scaling spheres and cylinders based off render. It could cause a bug where the user would enter a radius value that they probably want to be exact and then the render would average that input out to uniformly scale the object. So a user would enter a diameter of 10 and then the sphere would have a diameter of 3.33. To fix this I customized the scaling behavior with a new Vector3 class called scale vector. It is probablly not the most elegant solution but it is functional.
        function defineRenderBehavior(shape) {
            return () => {};
        }
    }

    get color() {
        return this.material.color;
    }

    get scale() {
        console.log("basic getter");
        return this._scale;
    }

    set color(color) {
        this.material.color.set(color);
    }

    getOffset = () => {
        return this.position;
    };

    addOffset(offset) {
        this.position.add(offset);
    }
    onBeforeRender = () => {
        Object.values(this.customRenderBehaviors).forEach((behavior) =>
            behavior(this)
        );
    };

    onAfterRender = () => {
        this.parent.parent.updateInertia();
    };
}
