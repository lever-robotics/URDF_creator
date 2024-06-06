import * as THREE from "three";
import UniformScaler from "./UniformScaler";
import UserData from "./UserData";
import Axis from "./Axis";
import Mesh from "./Mesh";

export default class SceneObject extends THREE.Object3D {
    constructor(shape, name, ...params) {
        super();

        // Two dictionaries of properties. Once dictionary can be assigned. The other must be set using the set() function
        const assignableProperties = {
            sceneObject: true,
            originOfRotation: new THREE.Object3D(),
            mesh: new Mesh(shape),
            uniformScaler: new UniformScaler(),
            jointAxis: new Axis({
                origin: params?.jointPosition ?? [0, 0, 0],
                axis: params?.jointAxis ?? [1, 0, 0],
                type: params?.jointType ?? "fixed",
                name: params?.jointName ?? "",
            }),
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

        // Must add these object in this order or it breaks??
        this.add(this.jointAxis);
        this.add(this.originOfRotation);
        this.originOfRotation.add(this.mesh);
        this.mesh.add(this.uniformScaler);
        this.mesh.add(this);

        // Settable properties through the .set() function
        this.position.set(...settableProperties.position);
        this.originOfRotation.rotation.set(...settableProperties.rotation);
        this.mesh.scale.set(...settableProperties.scale);
    }
    // --Joint
    //       |--Origin of Rotation
    //       |                   |--Mesh
    //       |                         |--Uniform Scaler
    //       |--Joint Axis

    getChildren = () => {
        console.log(this, this.uniformScaler)
        return this.uniformScaler.children;
    }

    // JS technically doesn't allow overloading but this seems to work haha
    add = (object) => super.add(object);
    addByUniformScaler = (object) => this.uniformScaler.add(object);
    attachByUniformScaler = (object) => this.uniformScaler.attach(object);

    // duplicate() {
    //     const copy = new THREE.Object3D();
    //     this.duplicateRecursive(this, copy);
    // }

    // duplicateRecursive(object, copy) {
    //     copy.originOfRotation = new THREE.Object3D();
    //     copy.mesh = object.mesh.duplicate()
    //     copy.uniformScaler = object.uniformScaler.duplicate()
    //     copy.jointAxis = object.jointAxis.duplicate();
    //     copy.children.push([copy.originOfRotation, copy.Axis]);
    //     copy.originOfRotation.children.push(copy.mesh);
    //     copy.mesh.children.push(copy.uniformScaler);
    //     copy.userData = object.userData.duplicate();
    // }
}
