import * as THREE from "three";
import UserData from "./UserData";
import Axis from "./Joint";
import Mesh from "./Mesh";
import Joint from "./Joint";
import Shimmy from "./Shimmy";

export default class urdfObject extends THREE.Object3D {
    constructor(shape, name, params) {
        super();

        // Two dictionaries of properties. Once dictionary can be assigned. The other must be set using the set() function
        const assignableProperties = {
            urdfObject: true, // Flag to determine if is urdfObject
            joint: new Joint(params),
            shimmy: new Shimmy(shape, params),
            userData: new UserData(shape, name),
        };
        const settableProperties = {
            position: params?.position ?? [0, 0, 0],
            rotation: params?.rotation ?? [0, 0, 0],
        };

        // Assign assignableProperties
        Object.entries(assignableProperties).forEach(([key, value]) => {
            this[key] = value;
        });

        // Add what is a property reference as a child
        this.add(this.joint);
        this.add(this.shimmy);

        // Settable properties through the .set() function
        this.position.set(...settableProperties.position);
        this.rotation.set(...settableProperties.rotation);
    }

    getChildren = () => {
        console.log(this, this.uniformScaler);
        return this.children;
    };

    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
}
