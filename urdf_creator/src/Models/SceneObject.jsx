import * as THREE from "three";
import UniformScaler from "./UniformScaler";
import UserData from "./UserData";
import Axis from "./Axis";
import Mesh from "./Mesh";

export default class SceneObject extends THREE.Object3D {
    constructor(shape, name) {
        super();
        this.sceneObject = true;

        this.originOfRotation = new THREE.Object3D();
        this.mesh = new Mesh(shape);
        this.uniformScaler = new UniformScaler();
        this.jointAxis = new Axis();

        this.add(this.originOfRotation);
        this.add(this.jointAxis);
        this.originOfRotation.add(this.mesh);

        this.mesh.addUniformScaler(this.uniformScaler);
        this.mesh.setJoint(this);

        this.userData = new UserData(shape, name);

        this.add = (object) => this.uniformScaler.add(object);
        this.attach = (object) => this.uniformScaler.attach(object);
    }
    // --Joint
    //       |--Origin of Rotation
    //       |                   |--Mesh
    //       |                         |--Uniform Scaler
    //       |--Joint Axis

    getChildren = () => this.uniformScaler.children;

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
