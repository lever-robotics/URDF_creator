import * as THREE from "three";
import { Color, Vector3 } from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { blobToArrayBuffer, getFile } from "../utils/localdb";
import type Frame from "./Frame";
import { Frameish } from "./Frame";
import ScaleVector from "./ScaleVector";

export default class VisualCollision extends THREE.Mesh {
    private _scale: ScaleVector;
    shape: string;
    frame!: Frame;
    material: THREE.MeshPhongMaterial;
    stlfile?: string;
    constructor(name: string, shape: string, scale: Vector3, color: number) {
        super();

        this.scale.copy(scale);

        this._scale = new ScaleVector(shape, scale);

        this.shape = shape;

        this.geometry = this.defineGeometry(shape);
        this.material = new THREE.MeshPhongMaterial();
        this.color = new Color(color);

        this.name = name;
    }

    private defineGeometry(shape: string) {
        switch (shape) {
            case "sphere":
                Object.defineProperty(this, "scale", {
                    get() {
                        return this._scale;
                    },
                    set(newVector) {
                        this._scale.set(...newVector);
                    },
                });

                return new THREE.SphereGeometry(0.5, 32, 32);

            case "cylinder": {
                Object.defineProperty(this, "scale", {
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
            case "mesh":
                Object.defineProperty(this, "scale", {
                    get() {
                        return this._scale;
                    },
                    set(newVector) {
                        this._scale.set(...newVector);
                    },
                });
                return new THREE.ConeGeometry(0.5, 1, 32); //temporarly show meshes as cones till develop mesh imports

            default: //Default to a Cube
                Object.defineProperty(this, "scale", {
                    get() {
                        return this._scale;
                    },
                    set(newVector) {
                        this._scale.set(...newVector);
                    },
                });

                return new THREE.BoxGeometry(1, 1, 1);
        }
    }

    setGeometry = async (geomertyType: string, fileName: string) => {
        //check if geometry type is cube, sphree, or cylinder
        if (
            geomertyType === "cube" ||
            geomertyType === "sphere" ||
            geomertyType === "cylinder"
        ) {
            //check if object is already that shape then do nothing
            if (this.shape === geomertyType) {
                return;
            }
            //set the shape to the new shape
            this.shape = geomertyType;
            //set the geometry to the new geometry
            this.scale.set(1, 1, 1);
            this.geometry = this.defineGeometry(geomertyType);
            this._scale.shape = geomertyType;
            console.log(this.scale);
            return;
        }
        //Then the geometry type is mesh
        this.shape = "mesh";
        this.objectScale.shape = "mesh";

        //check if object is already that shape
        if (this.stlfile === fileName) {
            return;
        }

        // Set the stlfile name to the userData
        this.stlfile = fileName;

        //get stl file from openDB
        try {
            // Get the STL file from IndexedDB
            const file = await getFile(fileName);

            if (file) {
                //convert the file to an array buffer
                const arrayBuffer = await blobToArrayBuffer(file);
                // Load the STL file
                // Create a Blob URL from the ArrayBuffer
                const blob = new Blob([arrayBuffer], {
                    type: "application/octet-stream",
                });
                const url = URL.createObjectURL(blob);

                // Load the STL file using STLLoader.load
                const loader = new STLLoader();
                loader.load(
                    url,
                    (geometry) => {
                        // Assign the loaded geometry to the current object
                        this.geometry = geometry;
                        this.shape = "mesh";

                        // Scale the mesh geometry to 0.001 on each axis
                        this.scale.set(0.001, 0.001, 0.001);

                        // Revoke the Blob URL after use
                        URL.revokeObjectURL(url);
                    },
                    undefined,
                    (error) => {
                        console.error("Error loading the STL file:", error);
                    },
                );
            } else {
                console.error("File not found in database");
            }
        } catch (error) {
            console.error("Error retrieving file from database:", error);
        }
    };

    get color() {
        return this.material.color;
    }

    get objectPosition() {
        return this.position;
    }

    get objectScale() {
        return this._scale;
    }

    get objectRotation() {
        return this.rotation;
    }

    get scaleVector() {
        return new THREE.Vector3(this._scale.x, this._scale.y, this._scale.z);
    }

    set color(color) {
        this.material.color.copy(color);
    }

    setColorByHex(hex: string): void {
        this.color = new THREE.Color(hex);
    }

    // duplicate() {
    //     const clone = new VisualCollision(this.name + "copy", this.shape, this.scale, this.color.getHex());
    //     clone.color.copy(this.color);
    //     // I added the as this. could potentially cause stupid errors
    //     return clone;
    // }

    onAfterRender = () => {
        this.frame?.updateInertia();
    };
}

export class Visual extends VisualCollision {
    constructor(
        number: number,
        shape = "cube",
        scale: Vector3 = new Vector3(1, 1, 1),
        color: number = Math.random() * 0xffffff,
    ) {
        // scale, color, "Visual" + number
        super(`Visual ${number}`, shape, scale, color);
    }

    duplicate() {
        const clone = new Visual(
            0,
            this.shape,
            this.scale,
            this.color.getHex(),
        );
        clone.color.copy(this.color);
        clone.name = `${this.name} copy`;
        return clone;
    }
}

export class Collision extends VisualCollision {
    constructor(
        number: number,
        shape = "mesh",
        scale: Vector3 = new Vector3(1, 1, 1),
        color = 0x808080,
    ) {
        super(`Collision ${number}`, shape, scale, color);
        //make the collision boxes transparent
        this.material.transparent = true;
        this.material.opacity = 0.5;
        this.material.wireframe = true;
    }

    duplicate() {
        const clone = new Collision(
            0,
            this.shape,
            this.scale,
            this.color.getHex(),
        );
        clone.color.copy(this.color);
        clone.name = `${this.name} copy`;
        return clone;
    }
}
