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
    sensor?: Sensor;
    inertia?: Inertia;
    jointVisualizer?: JointVisualizer;
    axis?: Axis;
    parentFrame?: Frame;
    
    linkDetached: boolean;
    isRootFrame: boolean;

    stlfile?: string;
    _jointType: string;

    constructor(
        public name = "",
        position: Vector3 = new Vector3(0, 0, 0),
        rotation: Euler = new Euler(0, 0, 0),
        jointType = "fixed",
        public min = -1,
        public max = 1
    ) {
        super();

        this.name = name;

        this.position.copy(position);
        this.rotation.copy(rotation);
        
        this.linkDetached = false;
        this.isRootFrame = false;

        this._jointType = jointType;
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
        return this.jointVisualizer!.children.filter(
            (child) => child instanceof Frame
        );
    };

    get parentName() {
        if (this.isRootFrame) return null;
        return this.parentFrame!.name;
    }

    get objectScale() {
        return this.mesh!.scale;
    }

    get objectPosition() {
        return this.position;
    }

    get objectRotation() {
        return this.rotation;
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

    set sensorType(type) {
        switch (type) {
            case "imu":
                this.sensor = new IMU();
                break;
            case "camera":
                this.sensor = new Camera();
                break;
            case "lidar":
                this.sensor = new Lidar();
                break;
            case "":
                this.sensor = new Sensor();
                break;
            // Add cases for other sensor types here
            default:
                throw Error("This type of sensor is not yet supported");
        }
    }

    updateInertia() {
        this.inertia!.updateInertia(this);
    }

    rotateAroundJointAxis = (angle: number) => {
        // Angle must be in radians
        // a quaternion is basically how to get from one rotation to another
        const quaternion = new THREE.Quaternion();

        // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
        quaternion.setFromEuler(this.axis!.rotation);

        // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

        // the joint's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer!.setRotationFromAxisAngle(newAxis, angle);
        
    };

    translateAlongJointAxis = (distance: number) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(this.axis!.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer!.position.set(0, 0, 0);
        this.jointVisualizer!.translateOnAxis(newAxis, distance);
    };

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

    duplicate() {
        return new Frame(
            this.name,
            this.position,
            this.rotation,
            this.jointType,
            this.min,
            this.max
        );
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
