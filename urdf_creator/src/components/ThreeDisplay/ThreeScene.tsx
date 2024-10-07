import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { Mouse } from "./Mouse";
import Frame, { Frameish } from "../../Models/Frame";

import { cloneFrame, createFrame, deregisterName, readScene, registerName, UserData } from "./TreeUtils";
import TransformControls from "../../Models/TransformControls";
import VisualCollision, {Collision, Visual} from "../../Models/VisualCollision";
import Inertia from "../../Models/Inertia";
import { CollisionData, VisualData } from "./TreeUtils";

type TransformControlsMode = "translate" | "rotate" | "scale";
export type Selectable = Frame | VisualCollision | Inertia;

export default class ThreeScene {
    worldFrame: Frame;
    rootFrame: Frameish;
    selectedObject: Frame | Visual | Collision | Inertia; //There is no joint type as when selected joint it is detaching the Frame from the link to be moved around.
    // selectedItem?: Frameish | Visual | Collision | Inertia; //There is no joint type as when selected joint it is detaching the Frame from the link to be moved around.
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
        this.worldFrame = new Frame();
        this.worldFrame.name = "world_frame";
        this.scene.add(this.worldFrame);
        this.rootFrame = null;
        this.selectedObject = this.worldFrame;
        this.toolMode = "translate";
        this.objectNames = [];
        this.numberOfShapes = {
            cube: 0,
            sphere: 0,
            cylinder: 0,
            mesh: 0
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

        // Filter for objects that are instances of the common base class VisualCollision or Frame
        const shapes: Selectable[] = intersects
            .filter((collision) => (collision.object instanceof VisualCollision || collision.object instanceof Frame))
            .map((collision) => collision.object as Selectable);


        // if we hit a shape, select the closest
        if (shapes.length > 0) {
            const object = shapes[0]
            if(object instanceof Frame){
                this.selectObject(object);
            }else{
                this.selectObject(object.frame!);
            }
            // if we don't hit any mesh 
        }else{
            // this.selectObject(null);
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
        this.transformControls.detach();
        this.rootFrame!.removeFromParent();
        this.rootFrame = null;
        this.objectNames.length = 0;
        this.selectedObject = this.worldFrame;
    };

    addObject = (shape: string) => {
        if (!this.scene) return;
        if (!(this.selectedObject instanceof Frame)) return;


        const numOfShape = (this.numberOfShapes[shape as keyof numShapes]).toString();
        const name = registerName(shape + numOfShape, this.objectNames);

        const newFrame = createFrame({
            shape: shape,
            name: name,
            collisions: [] as CollisionData[],
            visuals: [] as VisualData[],
        } as UserData);

        this.numberOfShapes[shape as keyof numShapes]++;

        newFrame.objectPosition.set(2.5, 2.5, 0.5);

        if (this.selectedObject.name === "world_frame") {
            if (this.rootFrame !== null) {
                this.rootFrame!.attachChild(newFrame);
            } else {
                newFrame.objectPosition.set(0, 0, 0.5);
                newFrame.isRootFrame = true;
                newFrame.name = "base_link";
                this.rootFrame = newFrame;
                this.worldFrame.attach(newFrame);
            }
        } else {
            this.selectedObject!.attachChild(newFrame);
        }
        this.selectObject(newFrame);
        this.forceUpdateCode();
        this.forceUpdateScene();
    };

    // TODO Close modal after loading scene
    loadScene = (gltfScene: THREE.Object3D) => {
        this.clearScene();
        const rootFrame = readScene(gltfScene, this.objectNames);

        this.worldFrame.attach(rootFrame);
        this.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;
        this.forceUpdateBoth();
    };

    setToolMode = (mode: string) => {
        
        if (this.transformControls) {
            this.transformControls.setMode(mode as TransformControlsMode);
            this.toolMode = mode as TransformControlsMode;
        }

        if (this.selectedObject) {
            this.attachTransformControls(this.selectedObject);
        }
        this.forceUpdateScene();
    };

    attachTransformControls = (object: Selectable) => {
        if(object.name === "world_frame") return;
        const selectedObject = object instanceof Frame
        const transformControls = this.transformControls;

        const mode = transformControls.mode;
        //depending on the selectedItem and the current mode either attach or not
        if(object instanceof Frame)
        // switch (mode) {
        //     // this case will attach the transform controls to the Frame and move everything together
        //     case "translate":
        //         transformControls.attach(selectedItem);
        //         break;
        //     // will attach to Frame which will rotate the mesh about said origin
        //     case "rotate":
        //         transformControls.attach(selectedItem);
        //         break;
        //     // will attach to the visual, collision, or inertia object but nothing else
        //     case "scale":
        //         if (selectedItem instanceof Visual || selectedItem instanceof Collision || selectedItem instanceof Inertia) {
        //             transformControls.attach(selectedItem!);
        //         }
        //         break;
        //     default:
        //         break;
        // }
        this.forceUpdateScene();
    };

    selectObject = (object: Selectable) => {
        if(!object){
            this.transformControls.detach();
            return;
        }
        if(object.name === "world_frame") return;


        // the link may not be attached correctly, this checks for that case
        // if (this.selectedObject?.linkDetached) {
        //     this.reattachLink(this.selectedObject!);
        // }
        
        this.selectedObject = object;
        this.transformControls.attach(object);
        // this.attachTransformControls(object);
        this.forceUpdateScene();
    };

    // selectItem = (item: Frameish | Visual | Collision | Inertia) => {
    //     this.selectedItem = item;
    //     //Get the frame of the item and set to selectedObject
    //     if (item instanceof Frame) {
    //         this.selectObject(item);
    //     } else {
    //         this.selectObject(item!.frame);
    //     }
    //     this.forceUpdateScene();
    // }

    loadSingleObject = (gltfScene: THREE.Object3D) => {
        const frame = readScene(gltfScene, this.objectNames);
        if (this.selectedObject instanceof Frame) {
            this.selectedObject.attachChild(frame);
        } else if (this.rootFrame) {
            this.rootFrame.attachChild(frame);
        } else {
            this.worldFrame.attach(frame);
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
        this.selectedObject = this.worldFrame;
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
}

type numShapes = {
    cube: number,
    sphere: number,
    cylinder: number,
    mesh: number
}
