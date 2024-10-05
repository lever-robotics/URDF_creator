import * as THREE from "three";
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import ThreeScene from "../components/ThreeDisplay/ThreeScene";

export default class TransformControl extends TransformControls{
    pointerMoved: boolean;
    scene: ThreeScene | undefined;

    constructor(camera: THREE.Camera, domElement: HTMLElement) {
        super(camera, domElement);
        this.pointerMoved = false;
        this.scene = undefined;
    }

    pointerDown(pointer: PointerEvent | null) {
        console.log(pointer);
        super.pointerDown(pointer);
        this.pointerMoved = true;
    }

    pointerMove(pointer: PointerEvent | null) {
        super.pointerMove(pointer);
        this.pointerMoved = false;
        this.scene!.forceUpdateScene();
    }

    pointerUp(pointer: PointerEvent | null) {
        super.pointerUp(pointer);
        
        if (this.pointerMoved) {
            console.log("upping");
            this.scene!.forceUpdateCode();
        }
    }
}
