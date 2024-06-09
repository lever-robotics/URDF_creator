import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor(urdfObject, params) {
        const jointAxis = {
            origin: [0, 0, 0],
            axis: [1, 0, 0],
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
        material.visible = false
        super(geometry, material);
        this.name = jointAxis.name;
        this.urdfObject = urdfObject;
        this.jointType = jointAxis.type;
        this.type = jointAxis.type;

    }

    set type (jointType){
        this.jointType = jointType;
        this.userData = { jointType: jointType};
    }

    get type () {
        return this.userData.jointType;
    }


}
