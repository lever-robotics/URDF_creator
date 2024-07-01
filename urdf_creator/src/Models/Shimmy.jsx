import * as THREE from "three";
import Link from "./Link";

export default class Shimmy extends THREE.Object3D {
    constructor(urdfObject, shape, params) {
        super();
        /* DESCRIPTION:
        Shimmy: Used to demonstrate joint logic and keep data on specific joint keyframes
        ------------------
        properties: Properties of Shimmy are the values/references to pertinent flags and user defined data.
            -> To modify the properties of Shimmy simply add the property name as the object key and the value as the value. A function later in the constructor will auto add these values to the object. 
        children: Direct children of Shimmy.
            -> Add direct children of Shimmy here. Their references will be automatically assigned as properties to the Shimmy. REMEMBER to use the add() function to add the references to the THREE.Object3D also
        attributes: These are the values that THREE function will directly modify to change the state of the scene.
            -> Add all attributes and their default values here and set them corresespondingly below
        */

        const properties = {};
        const children = {
            link: new Link(urdfObject, shape, params),
        };
        const attributes = {}; // The shimmy's position and rotation will be set but only for a visual feature to shimmy the joints

        //***Assign-add()-set()***//
        const assignProperties = (elements) => {
            // These are automatic
            Object.entries(elements).forEach(([key, value]) => {
                this[key] = value;
            });
        };
        assignProperties(properties);
        assignProperties(children);
        // Add Children here...
        this.add(this.link);
        // Add attributes here...
    }
    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
}
