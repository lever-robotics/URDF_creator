import Link from "./Link";
import Joint from "./Joint";
import Inertia from "./Inertia";
import Frame from "./Frame";
import { IMU, Camera, Lidar, Sensor } from "./SensorsClass";
import { sensorCreator } from "./SensorsClass";
import Axis from "./Axis";
import * as THREE from "three";
import Mesh from "./Mesh";

export default class FrameManager {
    constructor(stateFunctions) {
        this.stateFunctions = stateFunctions;
    }

    changeSensor(frame, type) {
        switch (type) {
            case "imu":
                frame.sensor = new IMU();
                break;
            case "camera":
                frame.sensor = new Camera();
                break;
            case "lidar":
                frame.sensor = new Lidar();
                break;
            case "":
                frame.sensor = new Sensor();
                break;
            // Add cases for other sensor types here
            default:
                throw Error("This type of sensor is not yet supported");
        }
    }

    createFrame(params) {
        const name = params.name;
        const shape = params.shape;

        const link = new Link();
        const joint = new Joint();
        const axis = new Axis();
        const mesh = new Mesh(shape);
        const inertia = new Inertia();
        const sensor = new Sensor();
        const unnamedFrame = new Frame(name);
        const frame = this.registerName(unnamedFrame);

        link.add(mesh);

        joint.link = link;
        joint.add(link);

        frame.joint = joint;
        frame.link = link;
        frame.axis = axis;
        frame.mesh = mesh;
        frame.add(joint);
        frame.add(axis);

        joint.frame = frame;
        link.frame = frame;
        axis.frame = frame;
        mesh.frame = frame;

        inertia.updateInertia(frame);
        frame.inertia = inertia;

        frame.sensor = sensor;
        return frame;
    }

    cloneFrame(frame) {
        const link = frame.link.clone();
        const joint = frame.joint.clone();
        const axis = frame.axis.clone();
        const mesh = frame.mesh.clone();
        const inertia = frame.inertia.clone();
        const sensor = frame.sensor.clone();

        const unnamedClone = frame.clone();
        const clone = this.registerName(unnamedClone);

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

        joint.frame = clone;
        link.frame = clone;
        axis.frame = clone;
        mesh.frame = clone;

        const children = frame.getFrameChildren();

        for (const child of children) {
            const cloneChild = this.cloneFrame(child);
            clone.addChild(cloneChild);
        }

        return clone;
    }

    // Recursively compress each Frame into a single mesh to make project storing as a gltf easier
    compressScene(frame) {
        const compressedFrame = new THREE.Mesh();
        const userData = {
            position: frame.position,
            rotation: frame.rotation,
            scale: frame.mesh.scale,
            offset: frame.link.position,
            jointType: frame.jointType,
            // jointAxis: joint.position,
            jointMin: frame.min,
            jointMax: frame.max,
            axisRotation: frame.axis.rotation,
            jointOrigin: frame.joint.position,
            material: frame.mesh.material,
            color: frame.color,
            shape: frame.shape,
            name: frame.name,
            mass: frame.mass,
            ixx: frame.inertia.ixx,
            ixy: frame.inertia.ixy,
            ixz: frame.inertia.ixz,
            iyy: frame.inertia.yyx,
            izz: frame.inertia.izz,
            iyz: frame.inertia.iyz,

            sensor: frame.sensor,
        };
        compressedFrame.userData = userData;

        frame.getFrameChildren().forEach((child) => {
            compressedFrame.add(this.compressScene(child));
        });

        return compressedFrame;
    }

    loadFrame(gltfObject) {
        const {
            position,
            rotation,
            scale,
            offset,
            jointType,
            jointMin,
            jointMax,
            axisRotation,
            jointOrigin,
            material,
            sensor,
            color,
            shape,
            name,
            mass,
            ixx,
            ixy,
            ixz,
            iyy,
            izz,
            iyz,
        } = gltfObject.userData;

        const link = new Link(Object.values(offset));
        const mesh = new Mesh(shape, Object.values(scale));
        const joint = new Joint(
            Object.values(jointOrigin),
            jointType,
            jointMin,
            jointMax
        );
        //Need to take out the nullish operator!
        const axis = new Axis(
            jointType,
            Object.values(axisRotation ?? {}).slice(1, 4)
        );
        axis.type = jointType;
        const inertia = new Inertia(mass, ixx, iyy, izz, ixy, ixz, iyz);

        const unnamedFrame = new Frame(
            name,
            Object.values(position),
            Object.values(rotation).slice(1, 4)
        );
        const frame = this.registerName(unnamedFrame);

        // BIG OLE COMMENT, this is a bandaid. Fix compressing and loading sensors
        frame.sensor = sensorCreator(sensor);

        link.mesh = mesh;
        link.add(mesh);

        joint.link = link;
        joint.add(link);

        frame.joint = joint;
        frame.link = link;
        frame.axis = axis;
        frame.mesh = mesh;
        frame.add(joint);
        frame.add(axis);
        frame.color = color;

        joint.frame = frame;
        link.frame = frame;
        axis.frame = frame;
        mesh.frame = frame;

        frame.inertia = inertia;
        return frame;
    }

    readScene(gltfObject) {
        const newFrame = this.loadFrame(gltfObject);

        gltfObject.children.forEach((child) => {
            const newChild = this.readScene(child);
            newFrame.addChild(newChild);
        });

        return newFrame;
    }

    registerName(frame) {
        // If the name is not in the array
        if (this.stateFunctions.registerName(frame.name)) {
            return frame;
        } else {
            //If it's in the array recursivly try to register the name
            let [name, suffix] = this.extractNumberFromString(frame.name);
            [name, suffix] = this.incrementName(name, suffix);
            frame.name = name + suffix;
            return this.registerName(frame);
        }
    }

    incrementName(name, suffix) {
        const isCopy = name.endsWith("-copy");
        if (isCopy && suffix) {
            suffix = (parseInt(suffix) + 1).toString();
        } else if (isCopy) {
            suffix = "0";
        } else {
            name = name + suffix + "-copy";
            suffix = "0";
        }
        if (this.stateFunctions.doesLinkNameExist(name + suffix)) {
            return this.incrementName(name, suffix);
        } else return [name, suffix];
    }

    extractNumberFromString(string, number = "") {
        const ending = string.slice(-1);
        // checks if it is a number
        if (!isNaN(ending)) {
            number = ending + number;
            return this.extractNumberFromString(
                string.slice(0, string.length - 1),
                number
            );
        } else return [string, number];
    }
}
