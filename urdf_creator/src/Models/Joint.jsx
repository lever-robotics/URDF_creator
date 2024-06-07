import * as THREE from "three";

export default class Joint extends THREE.Line {
    constructor({
        type = "fixed",
        axis = [1, 0, 0],
        origin = [0, 0, 0],
        name = "",
    } = {}) {
        const point = new THREE.Vector3(...origin);
        axis = new THREE.Vector3(...axis);
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
        const material = new THREE.LineBasicMaterial({ color: 0x0000ff });
        super(geometry, material);
        this.name = name;
        this.type = type;

        //             jointAxis: new Axis({
        //                 origin: params?.jointPosition ?? [0, 0, 0],
        //                 axis: params?.jointAxis ?? [1, 0, 0],
        //                 type: params?.jointType ?? "fixed",
        //                 name: params?.jointName ?? "",
        //             }),
    }

}
