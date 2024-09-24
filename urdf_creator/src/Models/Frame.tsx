import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { blobToArrayBuffer, getFile } from "../utils/localdb";
import Mesh from "./Mesh";
import { Camera, IMU, Lidar, Sensor } from "./SensorsClass";
import Inertia from "./Inertia";
import JointVisualizer from "./JointVisualizer";
import Link from "./Link";
import Axis from "./Axis";
import { Vector3, Euler } from "three";

export type Frameish = Frame | null | undefined;

export default class Frame extends THREE.Object3D {
    mesh?: Mesh;
    link?: Link;
    sensor?: Sensor | IMU | Camera | Lidar;
    inertia?: Inertia;
    jointVisualizer?: JointVisualizer;
    axis?: Axis;
    parentFrame?: Frame;
    linkDetached: boolean;
    name: string;
    stlfile?: string;
    selectable: boolean;
    isRootFrame: boolean;
    isFrame: boolean;
    frame: boolean;
    _jointType: string;
    _min: number;
    _max: number;

    

    constructor(name = "", position: Vector3 = new Vector3(0, 0, 0), rotation: Euler = new Euler(0, 0, 0), jointType = "fixed", jointMin = -1, jointMax = 1) {
        super();

        this.position.copy(position);
        this.rotation.copy(rotation);

        this._jointType = jointType;
        this._min = jointMin;
        this._max = jointMax;

        this.frame = true;
        this.isFrame = true;
        this.isRootFrame = false;
        this.selectable = true;
        this.name = name;
        this.linkDetached = false;
        // this.bus= [];
    }

    /**
     *
     *
     *
     *
     * GETTER/SETTER: Alphabetical
     *
     *
     *
     *
     **/

    getFrameChildren = () => {
        return this.jointVisualizer!.children.filter((child) => child instanceof Frame);
    };

    get parentName() {
        if (this.isRootFrame) return null;
        return this.parentFrame!.name;
    }

    get objectScale() {
        return this.mesh!.scaleVector;
    }

    get jointType() {
        return this._jointType;
    }

    set jointType(type) {
        this._jointType = type;
        switch (type) {
            case "fixed":
                this.axis!.material.visible = false;
                break;
            case "revolute":
            case "prismatic":
                this.min = -1;
                this.max = 1;
                this.axis!.material.visible = true;
                break;
            case "continuous":
                this.min = -3.14;
                this.max = 3.14;
                this.axis!.material.visible = true;
                break;
            default:
                break;
        }
    }

    get min() {
        return this._min;
    }

    set min(value) {
        this._min = value;
    }

    get max() {
        return this._max;
    }

    set max(value) {
        this._max = value;
    }

    get jointValue() {
        return this.jointVisualizer!.value;
    }

    set jointValue(value) {
        this.jointVisualizer!.value = value;
    }

    get offset() {
        return this.link!.position;
    }

    set mass(mass) {
        this.inertia!.updateMass(mass, this);
    }

    get mass() {
        return this.inertia!.mass;
    }

    get shape() {
        return this.mesh!.shape;
    }

    get color() {
        return this.mesh!.material.color;
    }

    setColorByHex(color: string) {
        this.mesh!.color = new THREE.Color(color);
    }

    get axisRotation() {
        return this.axis!.rotation;
    }

    set axisRotation(rotation: Euler) {
        this.axis!.rotation.copy(rotation);
    }

    attachChild(child: Frame) {
        this.jointVisualizer!.attach(child);
        child.parentFrame = this;
    }

    addChild(child: Frame) {
        this.jointVisualizer!.add(child);
        child.parentFrame = this;
    }

    get sensorType() {
        return this?.sensor?.type ?? "";
    }

    updateInertia() {
        this.inertia!.updateInertia(this);
    }

    setMesh = async (meshFileName: string) => {
        if (meshFileName === "") {
            this.mesh!.material.wireframe = false;
            this.userData.stlfile = null;
            this.link!.children = [];
            return;
        }

        //check if object already has a mesh and if so remove it and add the new mesh
        if (this.userData.stlfile === meshFileName) {
            return;
        }

        // Remove the existing mesh from link childreen
        if (this.jointVisualizer!.link!.children.length > 0) {
            this.jointVisualizer!.link!.children = [];
        }

        // Set the stlfile name to the userData
        this.userData.stlfile = meshFileName;

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
                            color: this.mesh!.color || Math.random() * 0xffffff,
                        });
                        const mesh = new THREE.Mesh(geometry, material);
                        // Compute the bounding box of the geometry
                        const boundingBox = new THREE.Box3().setFromObject(mesh);
                        // Define the desired bounding box dimensions
                        const desiredBox = new THREE.Box3(new THREE.Vector3(-0.5, -0.5, -0.5), new THREE.Vector3(0.5, 0.5, 0.5));

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

    duplicate() {
        return new Frame(this.name, this.position, this.rotation, this.jointType, this.min, this.max);
    }

    //Add STL to the Frame
    // setSTL = (stlfile) => {
    //     const loader = new STLLoader();
    //     loader.load(stlfile, (geometry) => {
    //         const material = new THREE.MeshPhongMaterial({ color: Math.random() * 0xffffff });
    //         const mesh = new THREE.Mesh(geometry, material);
    //         this.shimmy.link.mesh.add(mesh);
    //     });
    //     //make the mesh object a wireframe
    //     this.shimmy.link.mesh.children[0].material.wireframe = true;
    // }
}