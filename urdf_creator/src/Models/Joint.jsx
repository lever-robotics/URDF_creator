import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor(params) {
        const jointAxis = {
            origin: [0, 0, 0],
            axis: [0, 0, 1],
            type: params?.jointAxis?.type ?? "fixed",
            name: params?.jointName ?? "",
        };
        const point = new THREE.Vector3(...jointAxis.origin);
        const axis = new THREE.Vector3(...jointAxis.axis);
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
        const material = new THREE.LineBasicMaterial({ color: 0x00FFFF });
        material.visible = jointAxis.type !== "fixed";
        super(geometry, material);
        this.rotation.set(...params.jointRotation);
        this.name = jointAxis.name;
        this.userData = {
            jointType: jointAxis.type,
            min: params?.jointMin ?? -1,
            max: params?.jointMax ?? 1,
        }
        this.axis = axis;

    }

    set jointType(jointType){
        this.userData.jointType = jointType;
    }

    get jointType () {
        return this.userData.jointType;
    }

    set min (min) {
        this.userData.min = min;
        console.log(min);
    }

    get min() {
        return this.userData.min;
    }

    set max (max) {
        this.userData.max = max;
    }

    get max() {
        return this.userData.max;
    }

}
