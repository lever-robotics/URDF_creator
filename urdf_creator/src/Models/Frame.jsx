import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { blobToArrayBuffer, getFile } from "../utils/localdb";

export default class Frame extends THREE.Object3D {
    constructor(name = "", origin = [0, 0, 0], rotation = [0, 0, 0]) {
        super();

        this.position.set(...origin);
        this.rotation.set(...rotation);

        this.frame = true;
        this.isFrame = true;
        this.isBaseLink = false;
        this.selectable = true;
        this.stlfile = null;
        this.mesh = "";
        // if (stateFunctions.doesLinkNameExist(name)) {
        //     name += this.makeid(4)
        // }
        this.name = name;
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

    makeid(length) {
        let result = "";
        const characters = "0123456789";
        const charactersLength = characters.length;
        let counter = 0;
        while (counter < length) {
            result += characters.charAt(
                Math.floor(Math.random() * charactersLength)
            );
            counter += 1;
        }
        return result;
    }

    getFrameChildren = () => {
        return this.jointVisualizer.children.filter(
            (child) => child instanceof Frame
        );
    };

    get parentName() {
        if (this.isBaseLink) return null;
        return this.parentFrame.name;
    }

    get objectScale() {
        return this.mesh.scale;
    }

    get jointType() {
        return this.jointVisualizer.type;
    }

    set jointType(type) {
        this.jointVisualizer.type = type;
        this.axis.type = type;
    }

    get min() {
        return this.jointVisualizer.min;
    }

    set min(value) {
        this.jointVisualizer.min = value;
    }

    get max() {
        return this.jointVisualizer.max;
    }

    set max(value) {
        this.jointVisualizer.max = value;
    }

    get jointValue() {
        return this.jointVisualizer.value;
    }

    set jointValue(value) {
        this.jointVisualizer.value = value;
    }

    get offset() {
        return this.link.position;
    }

    set offset(values) {
        this.link.position.set(...values);
    }

    set mass(mass) {
        this.inertia.updateMass(mass, this);
    }

    get mass() {
        return this.inertia.mass;
    }

    get shape() {
        return this.mesh.shape;
    }

    get color() {
        return this.mesh.material.color;
    }

    set color(color) {
        this.mesh.color = color;
    }

    attachChild(child) {
        this.jointVisualizer.attach(child);
        child.parentFrame = this;
        // this.bus.push(child);
    }

    addChild(child) {
        this.jointVisualizer.add(child);
        child.parentFrame = this;
        // this.bus.push(child);
    }

    // removeChild(child) {
    //     const index = this.bus.indexOf(child);
    //     if (index > -1) {
    //         array.splice(index, 1);
    //     }
    // }

    setCustomInertia(type, inertia) {
        this.inertia.setCustomInertia(type, inertia);
    }

    updateInertia = () => {
        this.inertia.updateInertia(this);
    };

    updateMass = (mass) => {
        this.inertia.updateMass(mass, this);
    };

    // Angle must be in radians
    rotateAroundJointAxis(angle) {
        // a quaternion is basically how to get from one rotation to another
        const quaternion = new THREE.Quaternion();

        // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
        quaternion.setFromEuler(this.axis.rotation);

        // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

        // the joint's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer.setRotationFromAxisAngle(newAxis, angle);
    }

    translateAlongJointAxis(distance) {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(this.axis.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer.position.set(0, 0, 0);
        this.jointVisualizer.translateOnAxis(newAxis, distance);
    }

    resetJointPosition() {
        this.jointVisualizer.position.set(0, 0, 0);
        this.jointVisualizer.rotation.set(0, 0, 0);
    }

    get sensorType() {
        return this?.sensor?.type ?? "";
    }

    setMesh = async (meshFileName) => {
        if (meshFileName === "") {
            this.jointVisualizer.link.material.wireframe = false;
            this.userData.stlfile = null;
            this.link.children = [];
            return;
        }

        //check if object already has a mesh and if so remove it and add the new mesh
        if (this.userData.stlfile === meshFileName) {
            return;
        }

        // Remove the existing mesh from link childreen
        if (this.jointVisualizer.link.children.length > 0) {
            this.jointVisualizer.link.children = [];
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
                            color: this.link.color || Math.random() * 0xffffff,
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
                this.jointVisualizer.link.material.wireframe = true;
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
            // this case will attach the transform controls to the Frame and move everything together
            case "translate":
                transformControls.attach(this);
                break;
            // will attach to Frame which will rotate the mesh about said origin
            case "rotate":
                transformControls.attach(this);
                break;
            // will attach to the link and scale nothing else
            case "scale":
                transformControls.attach(this.mesh);
                break;
            default:
                break;
        }
    };

    // clear custom render behavior
    clearCustomRenderBehavior = (behavior) => {
        delete this.link.customRenderBehaviors[behavior];
    };

    operate = (type, axis, value) => {
        /* Rotation is a Euler object while Postion and Scale are Vector3 objects. To set all three properties in the same way I convert to an array first. */

        if (type === "offset") {
            const currentOffset = this.offset.toArray();
            this.parent.attach(this.link);
            this.linkDetached = true;

            const newValues = this.position.toArray();
            // console.log(newValues, currentOffset);
            newValues[this.determineComponentIndex(axis)] -=
                (value - currentOffset[this.determineComponentIndex(axis)]);
            this.position.set(...newValues);

            this.reattachLink();
        } else if (type === "scale") {
            const newValues = this.objectScale.toArray();
            newValues[this.determineComponentIndex(axis)] = value;
            this.objectScale.set(...newValues);
        } else {
            const newValues = this[type].toArray();
            newValues[this.determineComponentIndex(axis)] = value;
            this[type].set(...newValues);
        }
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

    // TODO rename this rotateJointAxis
    rotateJoint(transformControls) {
        transformControls.attach(this.axis);
    }

    moveJoint(transformControls) {
        this.parent.attach(this.link);
        // this.getFrameChildren.forEach((child) => {
        //     this.parent.attach(child);
        // })
        this.linkDetached = true;
        transformControls.attach(this);
    }

    reattachLink() {
        this.jointVisualizer.attach(this.link);
        this.linkDetached = false;
        // this.bus.forEach((child) => {
        //     this.jointVisualizer.attach(child);
        // })
        this.attach(this.axis);
    }

    clone() {
        return new Frame(this.name, this.position, this.rotation);
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
