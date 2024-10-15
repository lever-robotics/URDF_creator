import * as THREE from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import type { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import Frame, { type Shape, type Frameish } from "../../Models/Frame";
import type { Mouse } from "./Mouse";

import EventDispatcher, {
    type ListenerEvent,
    type ListenerEventLambda,
} from "../../Models/EventDispatcher";
import Inertia from "../../Models/Inertia";
import type TransformControls from "../../Models/TransformControls";
import VisualCollision, {
    Collision,
    Visual,
} from "../../Models/VisualCollision";
import {
    type UserData,
    cloneFrame,
    createFrame,
    deregisterName,
    readScene,
    registerName,
} from "./TreeUtils";
import type { CollisionData, VisualData } from "./TreeUtils";

export type TransformControlsMode = "translate" | "rotate" | "scale";
export type UserSelectable = Frame | Visual | Collision;
export type Selectable = Frame | Visual | Collision | null;
type EventType =
    | "updateCode"
    | "addObject"
    | "updateScene"
    | "toolMode"
    | "selectedObject"
    | "parameters"
    | "linkAttached"
    | "name"
    | "shape";
type numShapes = {
    cube: number;
    sphere: number;
    cylinder: number;
    mesh: number;
};

export default class ThreeScene {
    worldFrame: Frame;
    rootFrame: Frameish;
    selectedObject: Selectable; //There is no joint type as when selected joint it is detaching the Frame from the link to be moved around.
    // selectedItem?: Frameish | Visual | Collision | Inertia; //There is no joint type as when selected joint it is detaching the Frame from the link to be moved around.
    toolMode: TransformControlsMode;
    objectNames: string[];
    numberOfShapes: numShapes;
    eventDispatcher: EventDispatcher;
    linkDetached = false;

    constructor(
        public mountDiv: HTMLElement,
        public scene: THREE.Scene,
        public camera: THREE.Camera,
        public orbitControls: OrbitControls,
        public transformControls: TransformControls,
        public composer: EffectComposer,
        public raycaster: THREE.Raycaster,
        public mouse: Mouse,
        public callback: () => void,
    ) {
        this.worldFrame = new Frame();
        this.worldFrame.isWorldFrame = true;
        this.worldFrame.name = "origin";
        this.scene.add(this.worldFrame);
        this.rootFrame = null;
        this.selectedObject = this.worldFrame;
        this.toolMode = "translate";
        this.objectNames = [];
        this.numberOfShapes = {
            cube: 0,
            sphere: 0,
            cylinder: 0,
            mesh: 0,
        };
        this.mouse.addOnClickFunctions(this.clickObject);
        this.eventDispatcher = new EventDispatcher();
    }

    // Function added to the Mouse object to allow clicking of meshes
    clickObject = (event: PointerEvent) => {
        const rect = this.mountDiv.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;

        this.mouse.x = (x / rect.width) * 2 - 1;
        this.mouse.y = -(y / rect.height) * 2 + 1;

        this.raycaster.setFromCamera(
            new THREE.Vector2(this.mouse.x, this.mouse.y),
            this.camera,
        );
        const intersects = this.raycaster.intersectObjects(this.scene.children);
        console.log(intersects);
        // Filter for objects that are instances of the common base class VisualCollision or Frame
        const shapes: UserSelectable[] = intersects
            .filter(
                (collision) =>
                    collision.object instanceof VisualCollision ||
                    collision.object instanceof Frame,
            )
            .map((collision) => collision.object as UserSelectable);

        // if we hit a shape, select the closest
        if (shapes.length > 0) {
            const object = shapes[0];
            if (object instanceof Frame) {
                this.selectObject(object);
            } else {
                this.selectObject(object.frame);
            }
            // if we don't hit any mesh
        } else {
            this.selectObject(null);
        }
    };

    addEventListener(type: EventType, listener: ListenerEventLambda) {
        this.eventDispatcher.addEventListener(type, listener);
    }

    removeEventListener(type: EventType, listener: ListenerEventLambda) {
        this.eventDispatcher.removeEventListener(type, listener);
    }

    dispatchEvent(type: EventType) {
        const formattedEvent = {
            type: type,
            message: "",
        };
        this.eventDispatcher.dispatchEvent(formattedEvent);
    }

    forceUpdateCode = () => {
        this.dispatchEvent("updateCode");
        const customEvent = new Event("updateCode");
        this.mountDiv.dispatchEvent(customEvent);
    };

    clearScene = () => {
        if (!this.rootFrame) return;
        this.transformControls.detach();
        this.rootFrame.removeFromParent();
        this.rootFrame = null;
        this.objectNames.length = 0;
        this.selectedObject = null;
    };

    addObject = (shape: string) => {
        if (!this.scene) return;

        const numOfShape =
            this.numberOfShapes[shape as keyof numShapes].toString();
        const name = registerName(shape + numOfShape, this.objectNames);

        const newFrame = createFrame({
            shape: shape,
            name: name,
            collisions: [] as CollisionData[],
            visuals: [] as VisualData[],
        } as UserData);

        this.numberOfShapes[shape as keyof numShapes]++;

        newFrame.position.set(2.5, 2.5, 0.5);

        if (
            this.selectedObject === null ||
            (this.selectedObject instanceof Frame &&
                this.selectedObject.isWorldFrame)
        ) {
            if (this.rootFrame) {
                this.rootFrame.attachChild(newFrame);
            } else {
                newFrame.position.set(0, 0, 0.5);
                newFrame.isRootFrame = true;
                newFrame.name = "base_link";
                this.objectNames.push("base_link");
                newFrame.parentFrame = this.worldFrame;
                this.rootFrame = newFrame;
                this.worldFrame.attach(newFrame);
            }
        } else {
            if (this.selectedObject instanceof Frame) {
                this.selectedObject.attachChild(newFrame);
            } else {
                this.selectedObject.frame.attachChild(newFrame);
            }
        }
        this.selectObject(newFrame);
        this.dispatchEvent("addObject");
        this.forceUpdateCode();
    };

    addProperty = (shape: Shape, kind: "visual" | "collision") => {
        if (
            this.selectedObject instanceof Frame &&
            this.selectedObject.isWorldFrame
        )
            return;
        if (!this.selectedObject) return;
        const property =
            kind === "visual" ? new Visual(0, shape) : new Collision(0, shape);
        if (this.selectedObject instanceof Frame) {
            this.selectedObject.addProperty(property);
        } else {
            this.selectedObject.frame.addProperty(property);
        }
        this.forceUpdateCode();
    };

    // TODO Close modal after loading scene
    loadScene = (gltfScene: THREE.Object3D) => {
        this.clearScene();
        const rootFrame = readScene(gltfScene, this.objectNames);
        this.worldFrame.attach(rootFrame);
        rootFrame.parentFrame = this.worldFrame;
        this.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;
        this.forceUpdateCode();
    };

    setToolMode = (mode: TransformControlsMode) => {
        if (this.transformControls) {
            this.transformControls.setMode(mode);
            this.toolMode = mode;
        }

        this.dispatchEvent("toolMode");
    };

    selectObject = (object: Selectable) => {
        if (!object) {
            this.selectedObject = null;
            this.transformControls.detach();
            return;
        }
        if (object instanceof Frame && object.isWorldFrame) {
            this.selectedObject = object;
            this.transformControls.detach();
            this.dispatchEvent("selectedObject");
            return;
        }

        this.selectedObject = object;
        this.setToolMode("translate");
        this.transformControls.attach(object);
        // this.attachTransformControls(object);
        this.dispatchEvent("selectedObject");
    };

    loadSingleObject = (gltfScene: THREE.Object3D) => {
        const frame = readScene(gltfScene, this.objectNames);
        if (this.selectedObject instanceof Frame) {
            if (this.selectedObject.isWorldFrame) {
                this.addObject("cube"); // Make a base_link in case there isn't one
                this.selectedObject.attachChild(frame);
            }
        } else if (this.rootFrame) {
            this.rootFrame.attachChild(frame);
        } else {
            this.worldFrame.attach(frame);
            this.rootFrame = frame;
            frame.isRootFrame = true;
        }
        this.forceUpdateCode();
    };

    duplicateObject = (object: Selectable) => {
        if (!object) return;
        if (object instanceof Frame) {
            const clone = cloneFrame(object, this.objectNames);

            if (object.isRootFrame) {
                clone.parentFrame = object;
                object.attachChild(clone);
            } else {
                clone.parentFrame = object.parentFrame;
                object.parentFrame.addChild(clone);
            }
            this.selectObject(clone);
        } else {
            // If object is VisualCollision
            const clone = object.duplicate();
            object.frame.addProperty(clone);
            this.selectObject(clone);
        }

        this.forceUpdateCode();
    };

    deleteObject = (object: Selectable) => {
        this.transformControls.detach();
        if (!object) return;

        if (object instanceof VisualCollision) {
            object.frame.removeProperty(object);
            return;
        }

        const deleteChildren = (frame: Frame) => {
            for (const child of frame.getFrameChildren()) {
                deleteChildren(child);
                child.removeFromParent();
                deregisterName(child.name, this.objectNames);
            }
        };

        if (object.isRootFrame) {
            this.rootFrame = null;
        }
        this.selectedObject = this.worldFrame;
        deleteChildren(object);
        object.removeFromParent();
        deregisterName(object.name, this.objectNames);
        this.forceUpdateCode();
    };

    reparentObject = (parent: Frame, child: Frame) => {
        parent.attachChild(child);
        this.forceUpdateCode();
    };

    /*
        Frame Functions
    */
    startRotateJoint = (frame: Frame) => {
        this.transformControls.setMode("rotate");
        this.selectedObject = frame;
        this.transformControls.attach(frame.axis);
        this.linkDetached = true;
    };

    startMoveJoint = (frame: Frame) => {
        this.transformControls.setMode("translate");
        this.selectedObject = frame;
        frame.parentFrame.attach(frame.link);
        this.linkDetached = true;
        this.transformControls.attach(frame);
    };

    reattachLink = () => {
        this.transformControls.detach();
        if (!(this.selectedObject instanceof Frame) || !this.selectedObject)
            return;
        const frame = this.selectedObject;
        frame.jointVisualizer.attach(frame.link);
        this.linkDetached = false;
        frame.attach(frame.axis);
        this.dispatchEvent("linkAttached");
    };
}
