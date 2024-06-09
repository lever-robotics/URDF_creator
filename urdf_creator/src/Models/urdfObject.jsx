import * as THREE from "three";
import UserData from "./UserData";
import Joint from "./Joint";
import Shimmy from "./Shimmy";

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
           
           
        //***Properties-Children-Attributes***/
        const properties = {
            urdfObject: true, // Flag to determine if is urdfObject
            userData: new UserData(shape, name),
        };
        const children = {
            joint: new Joint(this, params),
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
        assignProperties(properties);
        assignProperties(children);
        // Add Children here...
        this.add(this.joint);
        this.add(this.shimmy);
        // Add direct attributes here...
        
        this.position.set(...attributes.position);
        this.rotation.set(...attributes.rotation);
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
        return this.shimmy.link.children;
    };

    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
}
