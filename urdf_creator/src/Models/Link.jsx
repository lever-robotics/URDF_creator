import * as THREE from "three";
import UniformScaler from "./UniformScaler";
import UserData from "./UserData";
import Axis from "./Axis";
import Mesh from "./Mesh";
import Joint from "./Joint";

export default class Link extends THREE.Object3D {
    constructor(shape, name, params) {
        super();

        // Two dictionaries of properties. Once dictionary can be assigned. The other must be set using the set() function
        const assignableProperties = {
            sceneObject: true,
            mesh: new Mesh(shape),
            joint: new Joint(params),
            userData: new UserData(shape, name),
        };
        const settableProperties = {
            position: params?.position ?? [0, 0, 0],
            rotation: params?.rotation ?? [0, 0, 0],
            scale: params?.scale ?? [1, 1, 1],
        };

        // Assign assignableProperties
        Object.entries(assignableProperties).forEach(([key, value]) => {
            this[key] = value;
        });

        this.add(this.mesh);

        // Settable properties through the .set() function
        this.position.set(...settableProperties.position);
        this.rotation.set(...settableProperties.rotation);
        this.mesh.scale.set(...settableProperties.scale);
    }

    getChildren = () => {
        console.log(this, this.uniformScaler)
        return this.children;
    }

    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
    // addByUniformScaler = (object) => this.uniformScaler.add(object);
    // attachByUniformScaler = (object) => this.uniformScaler.attach(object);

}