import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "../../Models/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./Mouse";
import Frame, { Frameish } from "../../Models/Frame";
import Link from "../../Models/Link";
import JointVisualizer from "../../Models/JointVisualizer";
import Inertia from "../../Models/Inertia";
import { Camera, IMU, Lidar, Sensor, sensorCreator} from "../../Models/SensorsClass";
import Axis from "../../Models/Axis";
import Mesh from "../../Models/Mesh";
import { TransformControlsMode } from "../../Models/TransformControls";

export default class ThreeScene {
    rootFrame: Frameish;
    selectedObject: Frameish;
    toolMode: TransformControlsMode;
    objectNames: string[];
    numberOfShapes: numShapes;

    constructor(
        public mountRef: React.RefObject<HTMLElement>,
        public scene: THREE.Scene,
        public camera: THREE.Camera,
        public orbitControls: OrbitControls,
        public transformControls: TransformControls,
        public composer: EffectComposer,
        public raycaster: THREE.Raycaster,
        public mouse: Mouse,
        public callback: () => void
    ) {
        this.rootFrame = null;
        this.selectedObject = null;
        this.toolMode = "translate";
        this.objectNames = [];
        this.numberOfShapes = {
            cube: 0,
            sphere: 0,
            cylinder: 0,
        }
    }

    forceUpdateCode = () => {
        // pushUndo();
        // redo.current = [];
        // setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    };

    forceSceneUpdate = () => {
        const customEvent = new Event("forceUpdate");
        this.mountRef.current?.dispatchEvent(customEvent);
        console.log("forceUpdate");
    }

    clearScene = () => {
        if (this.rootFrame === null) return;
        this.rootFrame!.removeFromParent();
        this.rootFrame = null;
        this.objectNames.length = 0;
        this.selectObject(null);
        this.forceSceneUpdate();
    };

    addObject = (shape: string) => {
        if (!this.scene) return;

        const newFrame = this.createFrame({
            shape: shape,
            name: shape + (this.numberOfShapes[shape as keyof typeof this.numberOfShapes] + 1).toString(),
        } as UserData);

        this.numberOfShapes[shape as keyof numShapes]++;

        newFrame.position.set(2.5, 2.5, 0.5);

        if (this.selectedObject) {
            this.selectedObject!.attachChild(newFrame);
        } else if (this.rootFrame !== null) {
            this.rootFrame!.attachChild(newFrame);
        } else {
            newFrame.position.set(0, 0, 0.5);
            newFrame.isRootFrame = true;
            this.setLinkName(newFrame, "base_link");
            this.rootFrame = newFrame;
            this.scene.attach(newFrame);
        }
        this.selectObject(newFrame);
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    // TODO Close modal after loading scene
    loadScene = (gltfScene: THREE.Object3D) => {
        this.objectNames.length = 0;
        const rootFrame = this.readScene(gltfScene);
        // if (three.rootFrame) {
        //     three.rootFrame.removeFromParent();
        // }
        this.scene.attach(rootFrame);
        this.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;
        this.forceSceneUpdate();
    };

    setTransformMode = (selectedObject: Frame, mode: string) => {
        if (this.transformControls) {
            this.transformControls.setMode(mode as TransformControlsMode);
            this.toolMode = mode as TransformControlsMode;
        }

        if (selectedObject) {
            this.attachTransformControls(selectedObject);
        }
        this.forceSceneUpdate();
    };

    attachTransformControls = (selectedObject: Frame) => {
        const transformControls = this.transformControls;

        const mode = transformControls.mode;
        switch (mode) {
            // this case will attach the transform controls to the Frame and move everything together
            case "translate":
                transformControls.attach(selectedObject);
                break;
            // will attach to Frame which will rotate the mesh about said origin
            case "rotate":
                transformControls.attach(selectedObject);
                break;
            // will attach to the link and scale nothing else
            case "scale":
                transformControls.attach(selectedObject.mesh!);
                break;
            default:
                break;
        }
        this.forceSceneUpdate();
    };

    getToolMode = () => {
        return this.toolMode;
    };

    selectObject = (frame: Frameish) => {

        // the link may not be attached correctly, this checks for that case
        if (this.selectedObject?.linkDetached) {
            this.reattachLink(this.selectedObject!);
        }

        if (!frame) {
            this.selectedObject = undefined;
            this.transformControls.detach();
        } else if (frame.selectable) {
            this.selectedObject = frame;
            this.attachTransformControls(frame);
        } else {
            this.selectedObject = undefined;
            this.transformControls.detach();
        }
        this.forceSceneUpdate();
    };

    loadSingleObject = (gltfScene: THREE.Object3D) => {
        const frame = this.readScene(gltfScene);
        if (this.selectedObject) {
            this.selectedObject.attachChild(frame);
        } else if (this.rootFrame) {
            this.rootFrame.attachChild(frame);
        } else {
            this.scene.attach(frame);
            this.rootFrame = frame;
            frame.isRootFrame = true;
        }
        this.forceSceneUpdate();
    };

    getScene = () => {
        return this.scene;
    };

    duplicateObject = (frame: Frame) => {
        const clone = this.cloneFrame(frame);

        if (frame.isRootFrame) {
            clone.parentFrame = frame;
            frame.attachChild(clone);
        } else {
            clone.parentFrame = frame.parentFrame;
            frame.parentFrame!.addChild(clone);
        }
        this.selectObject(clone);
        this.forceUpdateCode();
    };

    deleteObject = (frame: Frame) => {

        const deleteChildren = (frame: Frame) => {
            frame.getFrameChildren().forEach((child: Frame) => {
                deleteChildren(child);
                child.removeFromParent();
                this.deregisterName(child.name);
            });
        };

        if (frame.isRootFrame) {
            this.rootFrame = null;
        }
        this.selectObject(null);
        deleteChildren(frame);
        frame.removeFromParent();
        this.deregisterName(frame.name);
    };

    getRootFrame = () => {
        return this.rootFrame;
    };

    reparentObject = (parent: Frame, child: Frame) => {
        parent.attachChild(child);
        this.forceSceneUpdate();
    };

    readScene(gltfObject: THREE.Object3D) {
        const newFrame = this.createFrame(gltfObject.userData as UserData);

        gltfObject.children.forEach((child) => {
            const newChild = this.readScene(child);
            newFrame.addChild(newChild);
        });

        return newFrame;
    }

    createFrame(params: UserData): Frame {
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

    cloneFrame(frame: Frame): Frame {
        const link = frame.link!.duplicate();
        const jointVisualizer = frame.jointVisualizer!.duplicate();
        const axis = frame.axis!.duplicate();
        const mesh = frame.mesh!.duplicate();
        const inertia = frame.inertia!.duplicate();
        const sensor = frame.sensor!.duplicate();

        const unnamedClone = frame.duplicate();
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

    compressScene(frame: Frame): THREE.Mesh {
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
            material: frame.mesh!.material,
            color: frame.color,
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
            compressedFrame.add(this.compressScene(child));
        });

        return compressedFrame;
    }

    doesLinkNameExist = (name: string) => {
        return this.objectNames.includes(name);
    };

    setLinkName = (frame: Frame, name: string) => {
        // remove old name from registry
        this.deregisterName(frame.name);

        //add the new name
        frame.name = name;
        this.registerName(frame);
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    findFrameByName = (frame: Frame, name: string) => {
        if (frame.name === name) return frame;
        let returnChild = null;
        frame.getFrameChildren().forEach((child: Frame) => {
            returnChild = this.findFrameByName(child, name);
        });
        return returnChild;
    };

    registerName(frame: Frame): Frame {
        // If the name is not in the array
        if(!this.doesLinkNameExist(frame.name)){
            this.objectNames.push(frame.name);
            return frame;
        }else {
            //If it's in the array recursivly try to register the name
            let [name, suffix] = this.extractNumberFromString(frame.name);
            [name, suffix] = this.incrementName(name, suffix);
            frame.name = name + suffix;
            return this.registerName(frame);
        }
    }

    deregisterName = (name: string) => {
        const index = this.objectNames.indexOf(name);
        this.objectNames.splice(index, 1);
    };

    incrementName(name: string, suffix: string): [string, string] {
        const isCopy = name.endsWith("-copy");
        if (isCopy && suffix) {
            suffix = (parseInt(suffix) + 1).toString();
        } else if (isCopy) {
            suffix = "0";
        } else {
            name = name + suffix + "-copy";
            suffix = "0";
        }
        if (this.doesLinkNameExist(name + suffix)) {
            return this.incrementName(name, suffix);
        } else return [name, suffix];
    }

    extractNumberFromString(string: string, number: string = ""): [string, string] {
        const ending: string = string.slice(-1);
        // checks if it is a number
        if (/\d/.test(ending)) {
            number = ending + number;
            return this.extractNumberFromString(
            string.slice(0, string.length - 1),
            number
            );
        } else {
            return [string, number];
        }
    }


     /*
        Frame Functions
    */
    startRotateJoint = (frame: Frame) => {
        this.transformControls.setMode("rotate");
        this.transformControls.attach(frame.axis!);
        this.forceSceneUpdate();
    };

    startMoveJoint = (frame: Frame) => {
        this.transformControls.setMode("translate");

        frame.parent!.attach(frame.link!);
        frame.linkDetached = true;
        this.transformControls.attach(frame);
        this.forceSceneUpdate();
    };

    reattachLink = (frame: Frame) => {
        this.transformControls.detach();
        frame.jointVisualizer!.attach(frame.link!);
        frame.linkDetached = false;
        frame.attach(frame.axis!);
    };

    setLinkColor = (frame: Frame, color: string) => {
        frame.setColorByHex(color);
    };

    setMass = (frame: Frame, mass: number) => {
        frame.mass = mass;
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    setInertia = (frame: Frame, type: string, inertia: number) => {
        frame.inertia!.setCustomInertia(type, inertia);
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    setSensor = (frame: Frame, type: string) => {
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
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    updateSensor = (frame: Frame, name: string, value: string | number) => {
        frame.sensor!.update(name, value);
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    setJointType = (frame: Frame, type: string) => {
        frame.jointType = type;
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    setJointMinMax = (frame: Frame, type: string, value: number) => {
        switch (type) {
            case "both":
                frame.min = -value;
                frame.max = value;
                break;
            case "min":
                frame.min = value;
                break;
            case "max":
                frame.max = value;
        }
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    setJointValue = (frame: Frame, value: number) => {
        frame.jointValue = value;
    };

    rotateAroundJointAxis = (frame: Frame, angle: number) => {
        // Angle must be in radians
        // a quaternion is basically how to get from one rotation to another
        const quaternion = new THREE.Quaternion();

        // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
        quaternion.setFromEuler(frame.axis!.rotation);

        // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

        // the joint's rotation is then set to be a rotation around the new axis by this angle
        frame.jointVisualizer!.setRotationFromAxisAngle(newAxis, angle);
        
    };

    translateAlongJointAxis = (frame: Frame, distance: number) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(frame.axis!.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        frame.jointVisualizer!.position.set(0, 0, 0);
        frame.jointVisualizer!.translateOnAxis(newAxis, distance);
    };

    resetJointPosition = (frame: Frame) => {
        frame.jointVisualizer!.position.set(0, 0, 0);
        frame.jointVisualizer!.rotation.set(0, 0, 0);
        this.forceSceneUpdate();
    };

    setMesh = (frame: Frame, meshFileName: string) => {
        frame.setMesh(meshFileName);
        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    transformObject = (frame: Frame, transformType: string, axis: string, value: number) => {
        if (transformType === "scale") {
            const newValues = frame.objectScale.toArray();
            newValues[this.whichAxis(axis)] = value;
            frame.objectScale.set(...newValues);
        } else {
            switch (transformType) {
                case "position":
                    const newPos = frame.position.toArray();
                    newPos[this.whichAxis(axis)] = value;
                    frame.position.set(...newPos);
                    break;
                case "rotation":
                    const newRot = frame.rotation.toArray();
                    newRot[this.whichAxis(axis)] = value;
                    frame.rotation.set(...newRot);
                    break;
                case "scale":
                    const newScale = frame.objectScale.toArray();
                    newScale[this.whichAxis(axis)] = value;
                    frame.objectScale.set(...newScale);
                    break;
            }
        }

        this.forceUpdateCode();
        this.forceSceneUpdate();
    };

    whichAxis = (axis: string) => {
        switch (axis) {
            case "x":
            case "radius":
                return 0;
            case "y":
                return 1;
            case "z":
            case "height":
                return 2;
        }
        return 0;
    };

    setObjectPosition = (object: Frame, position: THREE.Vector3) => {
        object.position.copy(position);
        this.forceSceneUpdate();
    };

    setObjectScale = (object: Frame, scale: THREE.Vector3) => {
        object.scale.set(scale.x, scale.y, scale.z);
        this.forceSceneUpdate();
    };

    copyObjectScale = (object: Frame, scale: THREE.Vector3) => {
        object.scale.copy(scale);
        this.forceSceneUpdate();
    };

    setObjectQuaternion = (object: Frame, quaternion: THREE.Quaternion) => {
        object.quaternion.copy(quaternion);
        this.forceSceneUpdate();
    };


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

type numShapes = {
    cube: number,
    sphere: number,
    cylinder: number,
}
