import Link from "./Link";
import JointVisualizer from "./JointVisualizer";
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
        const {
            name,
            shape,
            version,
            position, 
            rotation, 
            jointType,
            jointMin,
            jointMax,
            axisRotation,
            offset, 
            scale, 
            material, 
            color, 
            mass, 
            ixx,
            ixy,
            ixz,
            iyy,
            izz,
            iyz,
            sensor,
        } = params;

        // Instantiate new objects
        const mesh = new Mesh(shape, scale, color);
        const link = new Link(offset);
        const jointVisualizer = new JointVisualizer();
        const axis = new Axis(axisRotation);
        const inertia = new Inertia(mass, ixx, iyy, izz, ixy, ixz, iyz);
        const unnamedFrame = new Frame(name, position, rotation, jointType, jointMin, jointMax);
        const frame = this.registerName(unnamedFrame);
        
        // Add link children
        link.add(mesh);
        
        // Add jV children
        jointVisualizer.add(link);
        jointVisualizer.link = link;
        
        // Add frame children
        frame.add(jointVisualizer);
        frame.add(axis);
        frame.jointVisualizer = jointVisualizer;
        frame.link = link;
        frame.axis = axis;
        frame.mesh = mesh;
        frame.sensor = sensorCreator(sensor);
        frame.inertia = inertia;
        inertia.updateInertia(frame);

        // Give all tree objects a reference to frame
        jointVisualizer.frame = frame;
        link.frame = frame;
        axis.frame = frame;
        mesh.frame = frame;

        return frame;
    }

    cloneFrame(frame) {
        const link = frame.link.clone();
        const jointVisualizer = frame.jointVisualizer.clone();
        const axis = frame.axis.clone();
        const mesh = frame.mesh.clone();
        const inertia = frame.inertia.clone();
        const sensor = frame.sensor.clone();

        const unnamedClone = frame.clone();
        const clone = this.registerName(unnamedClone);

        clone.link = link;
        clone.jointVisualizer = jointVisualizer;
        clone.axis = axis;
        clone.mesh = mesh;
        clone.inertia = inertia;
        clone.sensor = sensor;

        jointVisualizer.add(link);
        link.add(mesh);
        clone.add(jointVisualizer);
        clone.add(axis);

        jointVisualizer.frame = clone;
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
            name: frame.name,
            shape: frame.shape,
            version: "beta",
            position: frame.position,
            rotation: frame.rotation,
            jointType: frame.jointType,
            jointMin: frame.min,
            jointMax: frame.max,
            axisRotation: frame.axisRotation,
            offset: frame.offset,
            scale: frame.objectScale,
            material: frame.mesh.material,
            color: frame.color,
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

    readScene(gltfObject) {
        const newFrame = this.createFrame(gltfObject.userData);

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
