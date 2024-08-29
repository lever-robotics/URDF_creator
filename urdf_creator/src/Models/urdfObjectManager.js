import Link from "./Link";
import Joint from "./Joint";
import urdfObject from "./urdfObject";
import Inertia from "./Inertia";
import { IMU, Camera, Lidar, Sensor } from "./SensorsClass";
import Axis from "./Axis";
import * as THREE from "three";
import Mesh from "./Mesh";

export default class urdfObjectManager {
    changeSensor(urdfObject, type) {
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
            case "":
                urdfObject.sensor = new Sensor();
                break;
            // Add cases for other sensor types here
            default:
                throw Error("This type of sensor is not yet supported");
        }
    }

    createUrdfObject(params) {
        const name = params.name;
        const shape = params.shape;

        const link = new Link();
        const joint = new Joint();
        const axis = new Axis();
        const mesh = new Mesh(shape);
        const inertia = new Inertia();
        const sensor = new Sensor();
        const urdfobject = new urdfObject(name);

        link.add(mesh);

        joint.link = link;
        joint.add(link);

        urdfobject.joint = joint;
        urdfobject.link = link;
        urdfobject.axis = axis;
        urdfobject.mesh = mesh;
        urdfobject.add(joint);
        urdfobject.add(axis);

        joint.urdfObject = urdfobject;
        link.urdfObject = urdfobject;
        axis.urdfObject = urdfobject;
        mesh.urdfObject = urdfobject;

        inertia.updateInertia(urdfobject);
        urdfobject.inertia = inertia;

        urdfobject.sensor = sensor;
        return urdfobject;
    }

    cloneUrdfObject(urdfObject) {


        const link = urdfObject.link.clone();
        const joint = urdfObject.joint.clone();
        const axis = urdfObject.axis.clone();
        const mesh = urdfObject.mesh.clone();
        const inertia = urdfObject.inertia.clone();
        const sensor = urdfObject.sensor.clone();
        const clone = urdfObject.clone();

        clone.link = link;
        clone.joint = joint;
        clone.axis = axis;
        clone.mesh = mesh;
        clone.inertia = inertia;
        clone.sensor = sensor;

        joint.add(link);
        link.add(mesh);
        clone.add(joint);
        clone.add(axis);

        joint.urdfObject = clone;
        link.urdfObject = clone;
        axis.urdfObject = clone;
        mesh.urdfObject = clone;

        const children = urdfObject.getUrdfObjectChildren();

        for (const child of children) {
            clone.link.attach(this.cloneUrdfObject(child));
            child.parentURDF = clone;
        }

        return clone;
    }

    // Recursively compress each urdfObject into a single mesh to make project storing as a gltf easier
    compressScene(urdfObject) {
        const compressedObject = new THREE.Mesh();
        const userData = {
            position: urdfObject.position,
            rotation: urdfObject.rotation,
            scale: urdfObject.mesh.scale,
            offset: urdfObject.link.position,
            jointType: urdfObject.jointType,
            // jointAxis: joint.position,
            jointMin: urdfObject.min,
            jointMax: urdfObject.max,
            jointRotation: urdfObject.joint.rotation,
            jointOrigin: urdfObject.joint.position,
            material: urdfObject.mesh.material,
            shape: urdfObject.shape,
            name: urdfObject.name,
            mass: urdfObject.mass,
            ixx: urdfObject.inertia.ixx,
            ixy: urdfObject.inertia.ixy,
            ixz: urdfObject.inertia.ixz,
            iyy: urdfObject.inertia.yyx,
            izz: urdfObject.inertia.izz,
            iyz: urdfObject.inertia.iyz,
        };
        compressedObject.userData = userData;

        urdfObject.getUrdfObjectChildren().forEach((object) => {
            compressedObject.add(this.compressScene(object));
        });

        return compressedObject;
    }

    loadObject(gltfObject) {
        const { position, rotation, scale, offset, jointType, jointMin, jointMax, jointRotation, jointOrigin, material, shape, name, mass, ixx, ixy, ixz, iyy, izz, iyz } = gltfObject.userData;

        console.log(position);

        const link = new Link(Object.values(offset));
        const mesh = new Mesh(shape, Object.values(scale));
        const joint = new Joint(Object.values(jointOrigin), jointType, jointMin, jointMax);
        const axis = new Axis();
        const inertia = new Inertia(mass, ixx, iyy, izz, ixy, ixz, iyz);
        const sensor = new Sensor();
        const urdfobject = new urdfObject(name, Object.values(position), Object.values(rotation));

        link.mesh = mesh;
        link.add(mesh);

        joint.link = link;
        joint.add(link);

        urdfobject.joint = joint;
        urdfobject.link = link;
        urdfobject.axis = axis;
        urdfobject.mesh = mesh;
        urdfobject.add(joint);
        urdfobject.add(axis);

        joint.urdfObject = urdfobject;
        link.urdfObject = urdfobject;
        axis.urdfObject = urdfobject;
        mesh.urdfObject = urdfobject;

        urdfobject.inertia = inertia;

        urdfobject.sensor = sensor;
        return urdfobject;
    }

    readScene(gltfObject) {
        console.log(gltfObject);

        const newObject = this.loadObject(gltfObject);

        gltfObject.children.forEach((child) => {
            return newObject.add(this.readScene(child));
        });

        return newObject;
    }
}
