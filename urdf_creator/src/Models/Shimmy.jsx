import * as THREE from "three";
import UserData from "./UserData";
import Axis from "./Joint";
import Mesh from "./Mesh";
import Joint from "./Joint";
import Link from "./Link";

export default class Shimmy extends THREE.Object3D {
    constructor(shape, params) {
        // shape, name, ...params
        super();

        // Two dictionaries of properties. Once dictionary can be assigned. The other must be set using the set() function
        const assignableProperties = {
            link: new Link(shape, params),
        };

        Object.entries(assignableProperties).forEach(([key, value]) => {
            this[key] = value;
        });

        const settableProperties = {
            position: params?.position ?? [0, 0, 0],
            rotation: params?.rotation ?? [0, 0, 0],
        };

        this.add(this.link);

        // Settable properties through the .set() function
        this.position.set(...settableProperties.position);
        this.rotation.set(...settableProperties.rotation);
    }
    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
}
