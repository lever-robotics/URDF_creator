import * as THREE from "three";
import Mesh from "./Mesh";

export default class Link extends THREE.Object3D {
    constructor(shape, params) {
        super();
        /* DESCRIPTION:
        Link: Used to demonstrate joint logic and keep data on specific joint keyframes
        ------------------
        properties: Properties of Link are the values/references to pertinent flags and user defined data.
            -> To modify the properties of Link simply add the property name as the object key and the value as the value. A function later in the constructor will auto add these values to the object. 
        children: Direct children of Link.
            -> Add direct children of Link here. Their references will be automatically assigned as properties to the Link. REMEMBER to use the add() function to add the references to the THREE.Object3D also
        attributes: These are the values that THREE function will directly modify to change the state of the scene.
            -> Add all attributes and their default values here and set them corresespondingly below
        */
        const properties = {};
        const children = {
            mesh: new Mesh(shape, params),
        };
        const attributes = {
            offset: params?.offset ?? [0,0,0], // The offset from the joint
            // position: params?.position ?? [0, 0, 0],
        };

        const assignProperties = (elements) => {
            // These are automatic
            Object.entries(elements).forEach(([key, value]) => {
                this[key] = value;
            });
        };
        assignProperties(properties);
        assignProperties(children);
        // Add Children here...
        this.add(this.mesh);
        // Set attributes here
        this.position.set(...attributes.offset);
        // this.position.set(...attributes.position); // The offset from the joint
    }

    set offset (offset) {
        this.position.set(...offset);
    }

    getOffset = () => {
        return this.position;
    }

    addOffset (offset) {
        this.position.add(offset);
    }
    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
}