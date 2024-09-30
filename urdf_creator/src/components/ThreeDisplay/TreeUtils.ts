import Link from "../../Models/Link";
import JointVisualizer from "../../Models/JointVisualizer";
import Inertia from "../../Models/Inertia";
import Frame from "../../Models/Frame";
import { IMU, Camera, Lidar, Sensor, sensorCreator } from "../../Models/SensorsClass";
import Axis from "../../Models/Axis";
import * as THREE from "three";
import Mesh from "../../Models/Mesh";

export function createFrame(params: UserData): Frame {
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
    const frame = new Frame(name, position, rotation, jointType, jointMin, jointMax);
    
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
};

export function cloneFrame(frame: Frame, objectNames: string[]): Frame {
    const link = frame.link!.duplicate();
    const jointVisualizer = frame.jointVisualizer!.duplicate();
    const axis = frame.axis!.duplicate();
    const mesh = frame.mesh!.duplicate();
    const inertia = frame.inertia!.duplicate();
    const sensor = frame.sensor!.duplicate();
    const clone = frame.duplicate();
    clone.name = registerName(clone.name, objectNames);

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
        const cloneChild = cloneFrame(child, objectNames);
        clone.addChild(cloneChild);
    }

    return clone;
}

// Recursively compress each Frame into a single mesh to make project storing as a gltf easier
export function compressScene(frame: Frame): THREE.Mesh {
    const compressedFrame = new THREE.Mesh();
    const userData: UserData = {
        name: frame.name,
        shape: frame.shape,
        version: "beta",
        position: frame.objectPosition,
        rotation: frame.objectRotation,
        jointType: frame.jointType,
        jointMin: frame.min,
        jointMax: frame.max,
        axisRotation: frame.axisRotation,
        offset: frame.offset,
        scale: frame.objectScale,
        material: frame.mesh!.material,
        color: frame.color.getHex(),
        mass: frame.mass,
        ixx: frame.inertia!.ixx,
        ixy: frame.inertia!.ixy,
        ixz: frame.inertia!.ixz,
        iyy: frame.inertia!.iyy,
        izz: frame.inertia!.izz,
        iyz: frame.inertia!.iyz,
        sensor: frame.sensor,
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
    shape: string;
    version: string;
    position: THREE.Vector3;
    rotation: THREE.Euler;
    jointType: string;
    jointMin: number;
    jointMax: number;
    axisRotation: THREE.Euler;
    offset: THREE.Vector3;
    scale: THREE.Vector3;
    material: THREE.MeshPhongMaterial;
    color: number;
    mass: number;
    ixx: number;
    ixy: number;
    ixz: number;
    iyy: number;
    izz: number;
    iyz: number;
    sensor: Sensor | IMU | Camera | Lidar | undefined;
}