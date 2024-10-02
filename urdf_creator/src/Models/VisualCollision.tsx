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
    constructor(shape = "cube", scale: Vector3 = new Vector3(1, 1, 1), color = Math.random() * 0xffffff) {
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


    setMesh = async (meshFileName: string) => {
        if (meshFileName === "") {
            this.material.wireframe = false;
            this.stlfile = undefined;
            return;
        }

        //check if object already has a mesh and if so remove it and add the new mesh
        if (this.stlfile === meshFileName) {
            return;
        }

        // Set the stlfile name to the userData
        this.stlfile = meshFileName;

        // Add the STL Mesh to the Frame as a child of Frame.joint.link.mesh and apply wireframe to link geometry
        //get stl file from openDB
        try {
            // Get the STL file from IndexedDB
            const file = await getFile(meshFileName);

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
                        const material = new THREE.MeshPhongMaterial({
                            color: this.color || Math.random() * 0xffffff,
                        });
                        const mesh = new THREE.Mesh(geometry, material);
                        // Compute the bounding box of the geometry
                        const boundingBox = new THREE.Box3().setFromObject(
                            mesh
                        );
                        // Define the desired bounding box dimensions
                        const desiredBox = new THREE.Box3(
                            new THREE.Vector3(-0.5, -0.5, -0.5),
                            new THREE.Vector3(0.5, 0.5, 0.5)
                        );

                        // Calculate the size of the bounding box and desired box
                        const boundingBoxSize = new THREE.Vector3();
                        boundingBox.getSize(boundingBoxSize);
                        const desiredBoxSize = new THREE.Vector3();
                        desiredBox.getSize(desiredBoxSize);

                        // Calculate the scaling factor
                        const scaleX = desiredBoxSize.x / boundingBoxSize.x;
                        const scaleY = desiredBoxSize.y / boundingBoxSize.y;
                        const scaleZ = desiredBoxSize.z / boundingBoxSize.z;
                        const scale = Math.min(scaleX, scaleY, scaleZ);

                        // Apply the scaling to the mesh
                        mesh.scale.set(scale, scale, scale);

                        // Add the mesh to the scene
                        this.link!.add(mesh);

                        // Revoke the Blob URL after use
                        URL.revokeObjectURL(url);
                    },
                    undefined,
                    (error) => {
                        console.error("Error loading the STL file:", error);
                    }
                );

                // Make the mesh object a wireframe
                this.mesh!.material.wireframe = true;
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
    constructor(shape: string = "mesh", scale: Vector3 = new Vector3(1, 1, 1), color: number = Math.random() * 0xffffff) {
        super(shape, scale, color);
    }
}