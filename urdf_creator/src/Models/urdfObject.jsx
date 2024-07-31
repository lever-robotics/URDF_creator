import * as THREE from "three";
import Joint from "./Joint";
import Link from "./Link";

import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { openDB } from "idb";
import { blobToArrayBuffer, getFile } from "../utils/localdb";

export default class urdfObject extends THREE.Object3D {
    constructor(origin, rotation, name) {
        super();

        this.urdfObject = true;
        this.position.set(...origin);

        this.rotation.set(...rotation);

        this.name = name;
        this.isBaseLink = false;
        this.selectable = true;
        this.sensor = null;
        this.stlfile = null;
        this.material = null;
        this.mesh = "";

        Object.defineProperty(this, "scale", {
            get() {
                return this._scale;
            },
            set(newVector) {
                this._scale.set(...newVector);
            },
        });
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

    getUrdfObjectChildren = () => {
        return this.children.filter((child) => (child instanceof urdfObject));
    };

    // Set this urdfObject as the baseLink
    setAsBaseLink = (flag) => {
        this.isBaseLink = flag;
    };

    // Sets the inertia of the urdfObject in the userData Object in the Inertia object
    setInertia = (inertia) => {
        this.inertia = inertia;
        this.inertia.customInertia = true;
    };

    get parentName() {
        return this.parent.name;
    }

    get jointType() {
        return this.joint.type;
    }

    set jointType(type) {
        this.joint.type = type;
    }

    get min() {
        return this.joint.min;
    }

    set min(value) {
        this.joint.min = value;
    }

    get max() {
        return this.joint.max;
    }

    set max(value) {
        this.joint.max = value;
    }

    set mass(mass) {
        this.inertia.updateMass(mass, this);
    }

    get mass() {
        return this.inertia.mass;
    }

    get _scale() {
        return this.link.scale;
    }

    set _scale(newVector) {
        this.link.scale = newVector;
    }

    get shape() {
        return this.link.shape;
    }

    get color() {
        return this.link.material.color;
    }

    set color(color) {
        this.link.color = color;
    }

    setCustomInertia(type, inertia) {
        this.inertia.setCustomInertia(type, inertia);
    }

    // Updates the inertia object in the userData
    updateInertia = () => {
        this.inertia.updateInertia(this);
    };

    // update the mass stored in the object in the Inertia object
    updateMass = (mass) => {
        this.inertia.updateMass(mass, this);
    };

    // duplicate() {
    //     const duplicated = new UserData(this.shape);
    //     duplicated.stlfile = this.stlfile;
    //     duplicated.name = this.name + " copy";
    //     duplicated.inertia = this.inertia.duplicate();
    //     duplicated.isBaseLink = false;
    //     if (this.sensor) {
    //         duplicated.sensor = this.sensor.duplicate();
    //     }

    //     return duplicated;
    // }

    /*known bugs
    1. The blur effect that resets the position from the slider obviously will not work if you release hold of the slider and the mouse is no longer over that general element
    2. When releaseing the mouse after a change of origin or angle the joint is set just slightly off from the expected 0. Mayber because of old savedPosition/savedRotation values?
    */
    // Angle must be in radians
    rotateAroundJointAxis(angle) {
        const newRotation = this.joint.rotation.toArray();
        newRotation[2] = angle;
        console.log(newRotation);
        this.joint.rotation.set(...newRotation);
    }

    translateAlongJointAxis(distance) {
        this.joint.position.setZ(distance);
    }

    distanceAlongJointAxis() {
        if (this.joint.position.z <= this.joint.savedPosition.z) {
            return -this.joint.savedPosition.distanceTo(this.joint.position);
        }
        return this.joint.savedPosition.distanceTo(this.joint.position);
    }

    angleAroundJointAxis() {
        const rotation = new THREE.Vector3().setFromEuler(this.joint.rotation);
        const savedRotation = new THREE.Vector3().setFromEuler(
            this.joint.savedRotation
        );

        if (rotation.z <= savedRotation.z) {
            return -savedRotation.distanceTo(rotation);
        }
        return savedRotation.distanceTo(rotation);
    }

    saveForDisplayChanges() {
        this.joint.savedRotation.copy(this.joint.rotation);
        this.joint.savedPosition.copy(this.joint.position);
    }

    resetFromDisplayChanges() {
        this.joint.position.copy(this.joint.savedPosition);
        this.joint.rotation.copy(this.joint.savedRotation);
    }

    // Updates joint limits
    setJointLimits = (min = null, max = null) => {
        if (min !== null) {
            this.joint.min = min;
        }

        if (max !== null) {
            this.joint.max = max;
        }
        // this.clearShimmy();
    };

    // Get the parent urdfObject of this urdfObject. So not its direct THREE.js parent. That can be retrived by calling urdfObject.parent(). This function is to jump from urdfObject to urdfObject.
    getParent = () => {
        return this.parent;
    };

    // Get urdfObject's position
    getPosition() {
        return this.position;
    }

    // Set urdfObject's position
    setPosition(positionVector) {
        this.position.set(
            positionVector[0],
            positionVector[1],
            positionVector[2]
        );
    }

    get sensorType() {
        return this?.sensor?.type ?? "";
    }

    // Is the urdfObject selectable?
    isSelectable = () => {
        return this.selectable;
    };

    setMesh = async (meshFileName) => {
        if (meshFileName === "") {
            this.shimmy.link.material.wireframe = false;
            this.stlfile = null;
            return;
        }

        this.userData.stlfile = meshFileName;

        // Add the STL Mesh to the urdfObject as a child of urdfObject.shimmy.link.mesh and apply wireframe to Mesh
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
                            color:
                                this.link.color || Math.random() * 0xffffff,
                        });
                        const mesh = new THREE.Mesh(geometry, material);
                        // Compute the bounding box of the geometry
                        const boundingBox = new THREE.Box3().setFromObject(
                            mesh
                        );
                        console.log("Bounding Box:", boundingBox);

                        // Define the desired bounding box dimensions
                        const desiredBox = new THREE.Box3(
                            new THREE.Vector3(-0.5, -0.5, -0.5),
                            new THREE.Vector3(0.5, 0.5, 0.5)
                        );
                        console.log("Desired Box:", desiredBox);

                        // Calculate the size of the bounding box and desired box
                        const boundingBoxSize = new THREE.Vector3();
                        boundingBox.getSize(boundingBoxSize);
                        const desiredBoxSize = new THREE.Vector3();
                        desiredBox.getSize(desiredBoxSize);
                        console.log("Bounding Box Size:", boundingBoxSize);
                        console.log("Desired Box Size:", desiredBoxSize);

                        // Calculate the scaling factor
                        const scaleX = desiredBoxSize.x / boundingBoxSize.x;
                        const scaleY = desiredBoxSize.y / boundingBoxSize.y;
                        const scaleZ = desiredBoxSize.z / boundingBoxSize.z;
                        const scale = Math.min(scaleX, scaleY, scaleZ);
                        console.log("Scale Factor:", scale);

                        // Apply the scaling to the mesh
                        mesh.scale.set(scale, scale, scale);
                        console.log("Scaled Mesh:", mesh);

                        // Add the mesh to the scene
                        this.link.add(mesh);

                        // Revoke the Blob URL after use
                        URL.revokeObjectURL(url);
                    },
                    undefined,
                    (error) => {
                        console.error("Error loading the STL file:", error);
                    }
                );

                // Make the mesh object a wireframe
                this.link.mesh.material.wireframe = true;
            } else {
                console.error("File not found in database");
            }
        } catch (error) {
            console.error("Error retrieving file from database:", error);
        }
    };

    /**
     *
     *
     *
     *
     * Logic Functions: Alphabetical
     *
     *
     *
     *
     **/

    // add a custom render behavior to the link
    addCustomRenderBehavior = (behavior, func) => {
        this.link.customRenderBehaviors[behavior] = func;
    };

    // Attaches transform controls to the correct child
    attachTransformControls = (transformControls) => {
        const mode = transformControls.mode;
        switch (mode) {
            // this case will attach the transform controls to the urdfObject and move everything together
            case "translate":
                transformControls.attach(this);
                break;
            // will attach to urdfObject which will rotate the mesh about said origin
            case "rotate":
                transformControls.attach(this);
                break;
            // will attach to the link and scale nothing else
            case "scale":
                transformControls.attach(this.link);
                console.log(this.scale);
                break;
            default:
                break;
        }
    };

    // clear custom render behavior
    clearCustomRenderBehavior = (behavior) => {
        delete this.link.customRenderBehaviors[behavior];
    };

    // Clear's any shimmy position/rotation changes
    // clearShimmy = () => {
    //     this.shimmy.position.set(0,0,0);
    //     this.shimmy.rotation.set(0,0,0);
    // }

    // Clones the urdfObject and everything that goes with it. Children included
    // clone = () => {
    //     const params = {
    //         position: this.position,
    //         rotation: this.rotation,
    //         scale: this.mesh.scale,
    //         offset: this.link.getOffset(),
    //         jointAxis: {
    //             type: this.joint.type,
    //             axis: this.joint.axis,
    //             origin: [0, 0, 0], // Not sure how to do this
    //             name: this.joint.name,
    //         },
    //         jointOrigin: this.joint.position,
    //         material: this.mesh.material,
    //         shape: this.userData.shape,
    //         userData: this.userData,
    //         name: this.userData.name + "_copy",
    //     }
    //     return new urdfObject(params.shape, params.name, params);
    // }

    // Move the urdfObject exactly opposite the amount the link object moves in a direction to maintin the urdfObject origin.
    setGlobalPosition = (offsetPosition) => {
        // Get the current world matrix of the object
        const worldMatrix = new THREE.Matrix4();
        worldMatrix.copy(this.link.matrixWorld);

        // Extract the position, rotation, and scale from the world matrix
        const worldPosition = new THREE.Vector3();
        const worldRotation = new THREE.Quaternion();
        const worldScale = new THREE.Vector3();
        worldMatrix.decompose(worldPosition, worldRotation, worldScale);

        // Compute the difference between the new world position and the old world position
        const offset = offsetPosition.clone().sub(worldPosition);

        // Transform the offset by the inverse of the object's parent's world rotation
        // if (this.shimmy) {
        //     const parentWorldRotation = new THREE.Quaternion();
        //     this.shimmy.getWorldQuaternion(parentWorldRotation);
        //     parentWorldRotation.invert(); // Corrected method
        //     offset.applyQuaternion(parentWorldRotation);
        // }

        // Add the transformed offset to the object's local position
        this.link.addOffset(offset);
    };

    operate = (type, axis, value) => {
        /* Rotation is a Euler object while Postion and Scale are Vector3 objects. To set all three properties in the same way I convert to an array first. */
        const newValues = this[type].toArray();
        console.log(newValues);
        newValues[this.determineComponentIndex(axis)] = value;
        console.log(value);
        this[type].set(...newValues);
        console.log(newValues);
    };

    determineComponentIndex(axis) {
        try {
            switch (axis) {
                case "x":
                case "radius":
                    return 0;
                case "y":
                    return 1;
                case "z":
                case "height":
                    return 2;
                default:
                    throw new Error(
                        "Axis must be 'x', 'y', 'z', 'radius, or 'height'"
                    );
            }
        } catch (e) {
            console.error(e, "axis provided", axis);
        }
    }

    // Start rotating joint?
    rotateJoint(transformControls) {
        this.attach(this.link);
        transformControls.attach(this.joint);
    }

    moveJoint(transformControls) {
        this.attach(this.link);
        transformControls.attach(this.joint);
    }

    reattachLink() {
        this.joint.attach(this.link);
        this.remove(this.link);
    }

    //Add STL to the urdfObject
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
