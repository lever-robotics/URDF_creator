import * as THREE from "three";
import UserData from "./UserData";
import Joint from "./Joint";
import Shimmy from "./Shimmy";
import Mesh from "./Mesh";

export default class urdfObject extends THREE.Object3D {
    constructor(shape, name, params) {
        super();
        /* DESCRIPTION:
        urdfObject: The encompassing object. Contains children/grandchildren that makeup the Joint/Link logic of a URDF
        ------------------
        properties: Properties of the urdfObject are the values/references to pertinent flags and user defined data.
            -> To modify the properties of urdfObject simply add the property name as the object key and the value as the value. A function later in the constructor will auto add these values to the object. 
        children: Direct children of urdfObject.
            -> Add direct children of urdfObject here. Their references will be automatically assigned as properties to the urdfObject. REMEMBER to use the add() function to add the references to the THREE.Object3D also
        attributes: These are the values that THREE function will directly modify to change the state of the scene.
            -> Add all attributes and their default values here and set them corresespondingly below
        */

        // Properties
        this.urdfObject = true; // Flag to determine if is urdfObject
        this.userData = new UserData(shape, name);

        const children = {
            joint: new Joint(params),
            shimmy: new Shimmy(shape, params),
        };
        const attributes = {
            position: params?.position ?? [0, 0, 0],
            rotation: params?.rotation ?? [0, 0, 0],
        };

        const assignProperties = (elements) => {
            // These are automatic
            Object.entries(elements).forEach(([key, value]) => {
                this[key] = value;
            });
        };
        //***Assign-add()-set()***//
        assignProperties(children);
        // Add Children here...
        this.add(this.joint);
        this.add(this.shimmy);
        // Add direct attributes here...

        this.position.set(...attributes.position);
        this.rotation.set(...attributes.rotation);
    }

    // Get name of urdfObject from userData
    getName() {
        return this.userData.name;
    }
    // Set the name of the urdfObject via userData
    setName(name) {
        console.log(this);
        this.userData.name = name;
    }

    // Set the color of the mesh
    setColor = (color) => {
        this.mesh.material.color.set(color);
    }

    // Get urdfObject's position
    getPosition() {
        return this.position;
    }
    // Set urdfObject's position
    setPosition(positionVector) {
        this.position.set(positionVector[0], positionVector[1], positionVector[2]);
    }

    // Is the urdfObject the base link? Information stored in userData
    isBaseLink = () => {
        return this.userData.isBaseLink;
    }
    // Set if the urdfObject is the baseLink
    setAsBaseLink = (flag) => {
        this.userData.isBaseLink = flag;
    }

    get link() {
        return this.shimmy.link;
    }

    set link(link) {
        this.shimmy.link = link;
    }

    get mesh() {
        return this.shimmy.link.mesh;
    }

    set mesh(mesh) {
        this.shimmy.link.mesh = mesh;
    }

    get grandchildren() {
        return this.shimmy.link.children;
    }

    set grandchildren(object) {
        this.shimmy.link.children.push(object);
    }

    getChildren = () => {
        return this.shimmy.link.children.filter((child) => !(child instanceof Mesh));
    };

    getParent = () => {
        if(this.isBaseLink()){
            return this.parent;
        }else{
            return this.parent.parent.parent;
        }
    };

    // Adds a child to the urdfObject. This specifically gets added to the Link object.
    attachChild = (childObject) => {
        this.link.attach(childObject);
    }

    setInertia = (inertia) => {
        this.userData.inertia = inertia;
        this.userData.inertia.customInertia = true;
    }

    setSensor = (sensorObj) => {
        this.userData.sensor = sensorObj;
    }

    // Updates the inertia object in the userData
    updateInertia = () => {
        this.userData.inertia.updateInertia(this);
    }

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
            // will attach to the mesh and scale nothing else
            case "scale":
                transformControls.attach(this.mesh);
                break;
            default:
                break;
        }
    }

    // Operate on an object, either scale, position, or rotation
    operate = (type, x, y, z) => {
        switch (type) {
            case "scale":
                this.mesh.scale.set(x, y, z);
                //update the moment of inertia
                this.userData.inertia.updateInertia(this);
                break;
            case "position":
                this.position.set(x, y, z);
                break;
            case "rotation":
                this.rotation.set(x, y, z);
                break;
            default:
                return;
        }
    }

    // Clear's any shimmy position/rotation changes
    clearShimmy = () => {
        this.shimmy.position.set(0,0,0);
        this.shimmy.rotation.set(0,0,0);
    }

    // Start rotating joint?
    rotateJoint = (transformControls) => {
        this.clearShimmy();
        transformControls.attach(this.joint);
    }

    // add a custom render behavior to the mesh
    addCustomRenderBehavior = (behavior, func) => {
        this.mesh.customRenderBehaviors[behavior] = func;
    }
    // clear custom render behavior
    clearCustomRenderBehavior = (behavior) => {
        delete this.mesh.customRenderBehaviors[behavior];
    }

    // Get the worldPosition of the Link object
    linkWorldPosition = () => {
        return this.link.getWorldPosition(new THREE.Vector3());
    }

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
        if (this.shimmy) {
            const parentWorldRotation = new THREE.Quaternion();
            this.shimmy.getWorldQuaternion(parentWorldRotation);
            parentWorldRotation.invert(); // Corrected method
            offset.applyQuaternion(parentWorldRotation);
        }

        // Add the transformed offset to the object's local position
        this.link.addOffset(offset);
    };

    // Rotate the joint axis (which is stored in the Shimmy object)
    setJointAxisRotation = (angle) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(this.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(...this.joint.axis).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle.
        this.shimmy.setRotationFromAxisAngle(newAxis, angle);
    }

    // Move the joint axis (which is stored in the Shimmy object)
    setJointAxisPosition = (distance) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(this.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        this.shimmy.position.set(0, 0, 0);
        this.shimmy.translateOnAxis(newAxis, distance);
    }

    // Updates joint limits
    setJointLimits = (min = null, max = null) => {
        if (min !== null) {
            this.joint.min = min;
        }

        if (max !== null) {
            this.joint.max = max;
        }
        this.clearShimmy();
    }

    setJointType = (type) => {
        this.joint.jointType = type;
        this.clearShimmy();

        if (type === "fixed") {
            this.joint.material.visible = false;
        } else {
            this.joint.material.visible = true;
        }

        if (type === "prismatic") {
            this.joint.min = -1;
            this.joint.max = 1;
        } else if (type === "revolute") {
            this.joint.min = -3.14;
            this.joint.max = 3.14;
        }
    }

    // Is the urdfObject selectable?
    isSelectable = () => {
        return this.userData.selectable;
    }

    // update the mass stored in the userData object in the Inertia object
    updateMass = (mass) => {
        this.userData.inertia.updateMass(mass, this);
    }

    clone = () => {
        const params = {
            position: this.position,
            rotation: this.rotation,
            scale: this.mesh.scale,
            offset: this.link.getOffset(),
            jointAxis: {
                type: this.joint.type,
                axis: this.joint.axis,
                origin: [0, 0, 0], // Not sure how to do this
                name: this.joint.name,
            },
            jointOrigin: this.joint.position,
            material: this.mesh.material,
            shape: this.userData.shape,
            userData: this.userData,
            name: this.userData.name + "_copy",
        }
        return new urdfObject(params.shape, params.name, params);
    }



}
