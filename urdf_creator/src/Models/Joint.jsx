import * as THREE from "three";
import UniformScaler from "./UniformScaler";
import UserData from "./UserData";
import Axis from "./Axis";
import Mesh from "./Mesh";

export default class Joint extends THREE.Object3D {
    constructor(params) {
        // shape, name, ...params
        super();

        // Two dictionaries of properties. Once dictionary can be assigned. The other must be set using the set() function
        const assignableProperties = {
            sceneObject: true,
            jointAxis: new Axis({
                origin: params?.jointPosition ?? [0, 0, 0],
                axis: params?.jointAxis ?? [1, 0, 0],
                type: params?.jointType ?? "fixed",
                name: params?.jointName ?? "",
            }),
            // userData: new UserData(shape, name),
        };

        Object.entries(assignableProperties).forEach(([key, value]) => {
            this[key] = value;
        });
        const settableProperties = {
            position: params?.position ?? [0, 0, 0],
            rotation: params?.rotation ?? [0, 0, 0],
        };

        // Settable properties through the .set() function
        this.position.set(...settableProperties.position);
        this.rotation.set(...settableProperties.rotation);
    }
    // --Joint
    //       |--Origin of Rotation
    //       |                   |--Mesh
    //       |                         |--Uniform Scaler
    //       |--Joint Axis

    // JS technically doesn't allow overloading but this seems to work haha
    // add = (object) => super.add(object);
   
}