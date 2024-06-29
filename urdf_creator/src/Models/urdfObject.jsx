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
        return this.parent.parent.parent;
    };

    getParent = () => {
        return this.parent.parent.parent;
    };

    // Adds a child to the urdfObject. This specifically gets added to the Link object.
    attachChild = (childObject) => {
        this.link.attach(childObject);
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

    // Clear's any shimmy position/rotation changes
    clearShimmy = () => {
        this.shimmy.position.set(0,0,0);
        this.shimmy.rotation.set(0,0,0);
    }

    startRotateJoint = (transformControls) => {
        this.clearShimmy();
        transformControls.attach(this.joint);
    }
}
