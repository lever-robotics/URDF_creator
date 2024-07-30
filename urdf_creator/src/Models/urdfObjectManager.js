import Link from "./Link";
import Joint from "./Joint";
import urdfObject from "./urdfObject";
import Inertia from "./Inertia";
import { IMU, Camera, Lidar } from "./SensorsClass"

export default class urdfObjectManager {
    constructor(){

    }

    addSensor(urdfObject, type){
        switch (type) {
            case "imu":
                urdfObject.sensor = new IMU();
                break;
            case "camera":
                urdfObject.sensor = new Camera();
                break;
            case "lidar":
                urdfObject.sensor = new Lidar();
                break;
            // Add cases for other sensor types here
            default:
                throw Error("This type of sensor is not yet supported");
        }
    }

    createUrdfObject(params){
        const ps = {
            position: params?.position ?? [0,0,0],
            rotation: params?.rotation ?? [0,0,0],
            scale: params?.scale ?? [1,1,1],
            offset: params?.position ?? [0,0,0],
            jointAxis: {
                type: 'fixed',
                axis: [0,0,1],
                origin: [0, 0, 0], // Not sure how to do this
                name: "",
            },
            jointMin: params?.jointMin ?? -1,
            jointMax: params?.jointMax ?? 1,
            // jointRotation: ,
            // jointOrigin: ,
            // material: ,
            shape: params?.shape ?? "cube",
            // userData: ,
            name: params?.name ?? "",
        };
            
        const link = new Link(ps.offset,ps.shape, ps.scale);
        const joint = new Joint(ps.jointAxis.origin, ps.jointAxis.axis, ps.jointAxis.type, ps.jointMin, ps.jointMax);

        const urdfobject = new urdfObject(ps.position, ps.rotation, ps.name);
        joint.link = link;
        joint.add(link);
        urdfobject.joint = joint;
        urdfobject.link = link;
        urdfobject.add(joint);

        const inertia = new Inertia();
        inertia.updateInertia(urdfobject);
        urdfobject.inertia = inertia;


        return urdfobject;
    }

    readScene(gltfObject){
        // const shimmy = gltfObject.children[0] === THREE.Line ? gltfObject.children[0] : gltfObject.children[1];
        // const joint = gltfObject.children[0] === THREE.Line ? gltfObject.children[1] : gltfObject.children[0];
        // const link = shimmy.children[0];
        // const linkChildren = link.children;
        // const mesh = linkChildren.find((obj) => obj.type === "Mesh");
        // const params = {
        //     position: gltfObject.position,
        //     rotation: gltfObject.rotation,
        //     scale: mesh.scale,
        //     offset: link.position,
        //     jointAxis: {
        //         type: joint.userData?.jointType ?? 'fixed',
        //         axis: joint.position,
        //         origin: [0, 0, 0], // Not sure how to do this
        //         name: joint.name,
        //     },
        //     jointMin: joint.userData?.min,
        //     jointMax: joint.userData?.max,
        //     jointRotation: joint.rotation,
        //     jointOrigin: joint.position,
        //     material: mesh.material,
        //     shape: gltfObject.userData.shape,
        //     userData: gltfObject.userData,
        //     name: gltfObject.userData.name,
        // };
        // const children = link.children.map((object) => {
        //     if (object !== THREE.Mesh) {
        //         return object;
        //     }
        // });
        // const object = new urdfObject(params.shape, params.name, params);
        // children.forEach((child) => {
        //     if (child.type !== "Mesh") {
        //         return object.link.add(createUrdfObject(child));
        //     }
        // });
        // return object;
    }
}