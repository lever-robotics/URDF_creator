import * as THREE from "three";
import ScaleVector from "./ScaleVector";
import Frame, { Frameish } from "./Frame";
import { Color, Vector3 } from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { blobToArrayBuffer, getFile } from "../utils/localdb";

export default class VisualCollision extends THREE.Mesh {
    private _scale: ScaleVector;
    shape: string;
    customRenderBehaviors: {};
    frame: Frameish;
    material: THREE.MeshPhongMaterial;
    stlfile?: string;
    constructor(shape = "cube", scale: Vector3 = new Vector3(1, 1, 1), color: number) {
        super();

        this.scale.copy(scale);

        this._scale = new ScaleVector(shape, scale);

        this.shape = shape;

        this.geometry = defineGeometry(this, shape);
        this.material = new THREE.MeshPhongMaterial();
        this.color = new Color(color);

        this.customRenderBehaviors = {};

        function defineGeometry(context: VisualCollision, shape: string) {
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
                case "mesh":
                    Object.defineProperty(context, "scale", {
                        get() {
                            return this._scale;
                        },
                        set(newVector) {
                            this._scale.set(...newVector);
                        },
                    });

                    return new THREE.ConeGeometry(0.5, 1, 32); //temporarly show meshes as cones till develop mesh imports
            }
        }
    }


    setGeometry = async (geomertyType: string, fileName: string) => {
        //check if geometry type is cube, sphree, or cylinder
        if (geomertyType === "cube" || geomertyType === "sphere" || geomertyType === "cylinder") {
            //check if object is already that shape then do nothing
            if (this.shape === geomertyType) {
                return;
            }
            //set the shape to the new shape
            this.shape = geomertyType;
            //set the geometry to the new geometry
            if (geomertyType === "cube") {
                this.geometry = new THREE.BoxGeometry(1, 1, 1);
            } else if (geomertyType === "sphere") {
                this.geometry = new THREE.SphereGeometry(0.5, 32, 32);
            }
            else if (geomertyType === "cylinder") {
                const cylinder = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                cylinder.rotateX(Math.PI / 2);
                this.geometry = cylinder;
            }
            return;
        }
        //Then the geometry type is mesh
        this.shape = "mesh";

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
                    }
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
        return this.scale;
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

    duplicate(): this {
        const clone = new VisualCollision(this.shape, this.scale, this.color.getHex());
        clone.color.copy(this.color);
        // I added the as this. could potentially cause stupid errors
        return clone as this;
    }

    onAfterRender = () => {
        this.frame?.updateInertia();
    };
}

export class Visual extends VisualCollision {
    constructor(shape: string = "cube", scale: Vector3 = new Vector3(1, 1, 1), color: number = Math.random() * 0xffffff) {
        super(shape, scale, color);
    }
}

export class Collision extends VisualCollision {
    constructor(shape: string = "mesh", scale: Vector3 = new Vector3(1, 1, 1), color: number = 0x808080) {
        super(shape, scale, color);
        //make the collision boxes transparent
        this.material.transparent = true;
        this.material.opacity = 0.5;
        this.material.wireframe = true;
    }
}