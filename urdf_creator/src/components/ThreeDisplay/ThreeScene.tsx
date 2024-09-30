import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { Mouse } from "./Mouse";
import Frame, { Frameish } from "../../Models/Frame";
import Mesh from "../../Models/Mesh";

import { cloneFrame, createFrame, deregisterName, readScene, registerName, UserData } from "./TreeUtils";
import TransformControls from "../../Models/TransformControls";

type TransformControlsMode = "translate" | "rotate" | "scale";

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
        this.mouse.addOnClickFunctions(this.clickObject)
    }

    // Function added to the Mouse object to allow clicking of meshes
    clickObject = (event: MouseEvent) => {
        const rect = this.mountRef.current?.getBoundingClientRect();
        const x = event.clientX - rect!.left;
        const y = event.clientY - rect!.top;

        this.mouse.x = (x / rect!.width) * 2 - 1;
        this.mouse.y = -(y / rect!.height) * 2 + 1;

        this.raycaster.setFromCamera(new THREE.Vector2(this.mouse.x, this.mouse.y), this.camera);
        const intersects = this.raycaster.intersectObjects(this.scene.children);

        // this will contain all of our objects (they have the property "isShape")
        const shapes: Mesh[] = intersects.filter((collision) => collision.object instanceof Mesh)
                                            .map((collision) => collision.object as Mesh);


        // if we hit a shape, select the closest
        if (shapes.length > 0) {
            const object = shapes[0].frame;
            this.selectObject(object!);
            // if we don't hit any mesh 
        }else{
            this.selectObject(null);
        }
    }

    forceUpdateCode = () => {
        const customEvent = new Event("updateCode");
        this.mountRef.current?.dispatchEvent(customEvent);
    };

    forceUpdateScene = () => {
        const customEvent = new Event("updateScene");
        this.mountRef.current?.dispatchEvent(customEvent);
    }

    forceUpdateBoth = () => {
        this.forceUpdateCode();
        this.forceUpdateScene();
    }

    clearScene = () => {
        if (this.rootFrame === null) return;
        this.rootFrame!.removeFromParent();
        this.rootFrame = null;
        this.objectNames.length = 0;
        this.selectObject(null);
    };

    addObject = (shape: string) => {
        if (!this.scene) return;

        const numOfShape = (this.numberOfShapes[shape as keyof numShapes]).toString();
        const name = registerName(shape + numOfShape, this.objectNames);

        const newFrame = createFrame({
            shape: shape,
            name: name,
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
            newFrame.name = "base_link";
            this.rootFrame = newFrame;
            this.scene.attach(newFrame);
        }
        this.selectObject(newFrame);
        this.forceUpdateCode();
        this.forceUpdateScene();
    };

    // TODO Close modal after loading scene
    loadScene = (gltfScene: THREE.Object3D) => {
        this.clearScene();
        const rootFrame = readScene(gltfScene, this.objectNames);

        this.scene.attach(rootFrame);
        this.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;
        this.forceUpdateBoth();
    };

    setTransformMode = (selectedObject: Frame, mode: string) => {
        if (this.transformControls) {
            this.transformControls.setMode(mode as TransformControlsMode);
            this.toolMode = mode as TransformControlsMode;
        }

        if (selectedObject) {
            this.attachTransformControls(selectedObject);
        }
        this.forceUpdateScene();
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
        this.forceUpdateScene();
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
        this.forceUpdateScene();
    };

    loadSingleObject = (gltfScene: THREE.Object3D) => {
        const frame = readScene(gltfScene, this.objectNames);
        if (this.selectedObject) {
            this.selectedObject.attachChild(frame);
        } else if (this.rootFrame) {
            this.rootFrame.attachChild(frame);
        } else {
            this.scene.attach(frame);
            this.rootFrame = frame;
            frame.isRootFrame = true;
        }
        this.forceUpdateScene();
    };

    duplicateObject = (frame: Frame) => {
        const clone = cloneFrame(frame, this.objectNames);

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
                deregisterName(child.name, this.objectNames);
            });
        };

        if (frame.isRootFrame) {
            this.rootFrame = null;
        }
        this.selectObject(null);
        deleteChildren(frame);
        frame.removeFromParent();
        deregisterName(frame.name, this.objectNames);
        this.forceUpdateCode();
    };

    reparentObject = (parent: Frame, child: Frame) => {
        parent.attachChild(child);
        this.forceUpdateScene();
    };


     /*
        Frame Functions
    */
    startRotateJoint = (frame: Frame) => {
        this.transformControls.setMode("rotate");
        this.transformControls.attach(frame.axis!);
        this.forceUpdateScene();
    };

    startMoveJoint = (frame: Frame) => {
        this.transformControls.setMode("translate");

        frame.parent!.attach(frame.link!);
        frame.linkDetached = true;
        this.transformControls.attach(frame);
        this.forceUpdateScene();
    };

    reattachLink = (frame: Frame) => {
        this.transformControls.detach();
        frame.jointVisualizer!.attach(frame.link!);
        frame.linkDetached = false;
        frame.attach(frame.axis!);
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
}

type numShapes = {
    cube: number,
    sphere: number,
    cylinder: number,
}
