import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor(origin, ax, type, name) {

        const point = new THREE.Vector3(...origin);
        const axis = new THREE.Vector3(...ax);
        const length = 10;
        const startPoint = point
            .clone()
            .sub(axis.clone().multiplyScalar(length / 2));
        const endPoint = point
            .clone()
            .add(axis.clone().multiplyScalar(length / 2));
        const points = [];
        points.push(startPoint);
        points.push(endPoint);

        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
        material.visible = type !== "fixed";

        super(geometry, material);
        // this.urdfObject = urdfObject; // Grab a reference to the urdfObject
        // this.jointData = this.urdfObject.userData.jointData; // Grab a reference to the jointData
        // this.rotation.set(...(params?.jointRotation ?? [0, 0, 0]));
        // this.jointData.name = jointAxis.name;
        // this.jointData.type = jointAxis.type;
        // this.jointData.min = params?.jointMin ?? -1;
        // this.jointData.max = params?.jointMax ?? 1;
        // this.axis = axis;
    }

    set jointType(jointType) {
        this.jointData.type = jointType;
    }

    get jointType() {
        return this.jointData.type;
    }

    set min(min) {
        this.jointData.min = min;
        console.log(min);
    }

    get min() {
        return this.jointData.min;
    }

    set max(max) {
        this.jointData.max = max;
    }

    get max() {
        return this.jointData.max;
    }
}
