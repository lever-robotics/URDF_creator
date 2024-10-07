import Link from "../../Models/Link";
import JointVisualizer from "../../Models/JointVisualizer";
import Inertia from "../../Models/Inertia";
import Frame from "../../Models/Frame";
import { IMU, Camera, Lidar, Sensor, sensorCreator } from "../../Models/SensorsClass";
import Axis from "../../Models/Axis";
import * as THREE from "three";
import {Collision, Visual} from "../../Models/VisualCollision";

export function createFrame(params: UserData): Frame {
    const {
        name,
        version,
        shape, // This is use for generating a generic box, cylinder, or sphere
        position, 
        rotation, 
        jointType,
        jointMin,
        jointMax,
        axisRotation,
        offset, 
        mass, 
        ixx,
        ixy,
        ixz,
        iyy,
        izz,
        iyz,
        sensor,
        collisions,
        visuals,
    } = params;

    // Take in the shape if available and create a single collision and visual with that shape
    if(shape){
        collisions?.push({
            shape: shape,
            scale: new THREE.Vector3(1, 1, 1),
            color: 0x808080,
            position: new THREE.Vector3(0, 0, 0),
            rotation: new THREE.Euler(0, 0, 0),
        });
        visuals?.push({
            shape: shape,
            scale: new THREE.Vector3(1, 1, 1),
            color: Math.random() * 0xffffff,
            position: new THREE.Vector3(0, 0, 0),
            rotation: new THREE.Euler(0, 0, 0),
        });
    }

    // Map all collisions to objects that will be attached to the link
    const collisionObjects = collisions?.map((collision, index) => {
        return new Collision(index, collision.shape, collision.scale, collision.color);
    }) || [];

    // Map all visuals to objects that will be attached to the link
    const visualObjects = visuals?.map((visual, index) => {
        return new Visual(index, visual.shape, visual.scale, visual.color);
    }) || [];

    // Instantiate new objects
    const link = new Link(offset);
    const jointVisualizer = new JointVisualizer();
    const axis = new Axis(axisRotation);
    const frame = new Frame(name, position, rotation, jointType, jointMin, jointMax);
    const inertia = new Inertia(frame, mass, ixx, iyy, izz, ixy, ixz, iyz);

    // Add link children
    collisionObjects.forEach((collision) => link.add(collision));
    visualObjects.forEach((visual) => link.add(visual));
    
    // Add jV children
    jointVisualizer.add(link);
    jointVisualizer.link = link;
    
    // Add frame children
    frame.add(jointVisualizer);
    frame.add(axis);
    frame.jointVisualizer = jointVisualizer;
    frame.link = link;
    frame.axis = axis;
    frame.visuals = visualObjects;
    frame.collisions = collisionObjects;
    frame.sensor = sensorCreator(sensor);
    frame.inertia = inertia;
    inertia.updateInertia(frame);

    // Give all tree objects a reference to frame
    jointVisualizer.frame = frame;
    link.frame = frame;
    axis.frame = frame;
    visualObjects.forEach((visual) => visual.frame = frame);
    collisionObjects.forEach((collision) => collision.frame = frame);

    return frame;
};

export function cloneFrame(frame: Frame, objectNames: string[]): Frame {
    const link = frame.link!.duplicate();
    const jointVisualizer = frame.jointVisualizer!.duplicate();
    const axis = frame.axis!.duplicate();
    const visuals = frame.visuals!.map((visual) => visual.duplicate());
    const collisions = frame.collisions!.map((collision) => collision.duplicate());
    const inertia = frame.inertia!.duplicate();
    const sensor = frame.sensor!.duplicate();
    const clone = frame.duplicate();
    clone.name = registerName(clone.name, objectNames);

    clone.link = link;
    clone.jointVisualizer = jointVisualizer;
    clone.axis = axis;
    clone.visuals = visuals;
    clone.collisions = collisions;
    clone.inertia = inertia;
    clone.sensor = sensor;

    jointVisualizer.add(link);
    visuals.forEach((visual) => link.add(visual));
    collisions.forEach((collision) => link.add(collision));
    clone.add(jointVisualizer);
    clone.add(axis);

    jointVisualizer.frame = clone;
    link.frame = clone;
    axis.frame = clone;
    visuals.forEach((visual) => visual.frame = clone);
    collisions.forEach((collision) => collision.frame = clone);

    const children = frame.getFrameChildren();

    for (const child of children) {
        const cloneChild = cloneFrame(child, objectNames);
        clone.addChild(cloneChild);
    }

    return clone;
}

// Recursively compress each Frame into a single mesh to make project storing as a gltf easier
export function compressScene(frame: Frame): THREE.Mesh {
    const compressedFrame = new THREE.Mesh();
    
    const collisions = frame.collisions?.map((collision) => {
        return {
            shape: collision.shape,
            scale: collision.scale,
            color: collision.color.getHex(),
            position: collision.position,
            rotation: collision.rotation,
        };
    }) || [];

    const visuals = frame.visuals?.map((visual) => {
        return {
            shape: visual.shape,
            scale: visual.scale,
            color: visual.color.getHex(),
            position: visual.position,
            rotation: visual.rotation,
        };
    }) || [];

    const userData: UserData = {
        name: frame.name,
        version: "beta2",
        position: frame.objectPosition,
        rotation: frame.objectRotation,
        jointType: frame.jointType,
        jointMin: frame.min,
        jointMax: frame.max,
        axisRotation: frame.axisRotation,
        offset: frame.offset,
        mass: frame.mass,
        ixx: frame.inertia!.ixx,
        ixy: frame.inertia!.ixy,
        ixz: frame.inertia!.ixz,
        iyy: frame.inertia!.iyy,
        izz: frame.inertia!.izz,
        iyz: frame.inertia!.iyz,
        sensor: frame.sensor,
        collisions: collisions,
        visuals: visuals,
    };
    compressedFrame.userData = userData;

    frame.getFrameChildren().forEach((child) => {
        compressedFrame.add(compressScene(child));
    });

    return compressedFrame;
}

export function readScene(gltfObject: THREE.Object3D, objectNames: string[]) {
    const userData: UserData = gltfObject.userData as UserData;
    const registeredName = registerName(userData.name, objectNames);
    userData.name = registeredName;

    const newFrame = createFrame(userData);


    gltfObject.children.forEach((child) => {
        const newChild = readScene(child, objectNames);
        newFrame.addChild(newChild);
    });

    return newFrame;
}

export function registerName(name: string, objectNames: string[]): string {
    const doesLinkNameExist = objectNames.includes(name);
    // If the name is not in the array
    if(!doesLinkNameExist){
        objectNames.push(name);
        return name;
    }else {
        //If it's in the array recursivly try to register the name
        const incrementedName = incrementName(name);
        return registerName(incrementedName, objectNames);
    }
}

export function deregisterName(name: string, objectNames: string[]){
    const index = objectNames.indexOf(name);
    objectNames.splice(index, 1);
};

export function findFrameByName(frame: Frame, name: string){
    if (frame.name === name) return frame;
    let returnChild = null;
    frame.getFrameChildren().forEach((child: Frame) => {
        returnChild = findFrameByName(child, name);
    });
    return returnChild;
};

function incrementName(name: string): string {
    const [nameWithoutNum, suffix] = extractNumberFromString(name)
    const incrementedSuffix = Number(suffix) + 1;
    return nameWithoutNum + incrementedSuffix;
}

function extractNumberFromString(str: string): [string, string] {
    const suffix: string[] | null = str.match(/\d+/g);
    const stringWithoutNums = str.replace(/\d+/g, '');

    if(suffix === null){
        return [stringWithoutNums, ""];
    }else{
        return [stringWithoutNums, suffix.pop()!];
    }
}


export type UserData = {
    name: string;
    shape?: string;
    version: string;
    position: THREE.Vector3;
    rotation: THREE.Euler;
    jointType: string;
    jointMin: number;
    jointMax: number;
    axisRotation: THREE.Euler;
    offset: THREE.Vector3;
    mass: number;
    ixx: number;
    ixy: number;
    ixz: number;
    iyy: number;
    izz: number;
    iyz: number;
    sensor: Sensor | IMU | Camera | Lidar | undefined;
    collisions?: CollisionData[];
    visuals?: VisualData[];
}

export type CollisionData = {
    shape: string;
    scale: THREE.Vector3;
    color: number; 
    position: THREE.Vector3;
    rotation: THREE.Euler;
};

export type VisualData = {
    shape: string;
    scale: THREE.Vector3;
    color: number; 
    position: THREE.Vector3;
    rotation: THREE.Euler
};