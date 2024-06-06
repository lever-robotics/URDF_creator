import * as THREE from "three";

export default class Mesh extends THREE.Mesh {
    constructor(shape) {
        let geometry;
        let customRenderBehavior = () => {};
        switch (shape) {
            case "cube":
                geometry = new THREE.BoxGeometry(1, 1, 1);
                break;
            case "sphere":
                geometry = new THREE.SphereGeometry(0.5, 32, 32);
                // ensure spheres scale uniformly in all directions
                customRenderBehavior = function (renderer, scene, camera, geometry, material, group) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale = (worldScale.x + worldScale.y + worldScale.z) / 3;

                    const localScale = this.scale;
                    this.scale.set((localScale.x / worldScale.x) * uniformScale, (localScale.y / worldScale.y) * uniformScale, (localScale.z / worldScale.z) * uniformScale);
                };
                break;
            case "cylinder":
                geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                // ensure cylinders scale uniformly in two directions
                customRenderBehavior = function (renderer, scene, camera, geometry, material, group) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale = (worldScale.x + worldScale.z) / 2;

                    const localScale = this.scale;
                    this.scale.set((localScale.x / worldScale.x) * uniformScale, localScale.y, (localScale.z / worldScale.z) * uniformScale);
                };
                break;
            default:
                return;
        }

        const material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });

        super(geometry, material);
        this.customRenderBehavior = customRenderBehavior;
        this.isShape = true;



    }

    onBeforeRender = (renderer, scene, camera, geometry, material, group) => {
        // this.uniformScaler.doScale();
        this.customRenderBehavior(renderer, scene, camera, geometry, material, group);
    };

    addUniformScaler = (scaler) => {
        super.add(scaler);
        this.uniformScaler = scaler;
    };

    add = (property) => {
        const className = property.constructor.name;
        console.log(className);
        if(className === "SceneObject"){
            this.setJoint(property);
        }else if(className === "UniformScaler"){
            this.addUniformScaler(property);
        }
    }

    setJoint = (joint) => {
        this.joint = joint;
    };



}
