import * as THREE from 'three';

export default class UniformScaler extends THREE.Object3D {
    constructor() {
        super();
        this.userData.selectable = false;
        this.userData.scaler = true;
        this.doScale = (renderer, scene, camera, geometry, material, group) => {
            const worldScale = new THREE.Vector3();
            this.getWorldScale(worldScale);
            const uniformScale = 1;
            const localScale = this.scale;
            this.scale.set((localScale.x / worldScale.x) * uniformScale, (localScale.y / worldScale.y) * uniformScale, (localScale.z / worldScale.z) * uniformScale);
        };
    }
}