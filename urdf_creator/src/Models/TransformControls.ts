import type * as THREE from "three";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import type ThreeScene from "../components/ThreeDisplay/ThreeScene";

export default class TransformControl extends TransformControls {
    pointerMoved: boolean;
    scene!: ThreeScene;

    constructor(camera: THREE.Camera, domElement: HTMLElement) {
        super(camera, domElement);
        this.pointerMoved = false;
    }

    pointerDown(pointer: PointerEvent | null) {
        console.log(pointer);
        super.pointerDown(pointer);
    }

    pointerMove(pointer: PointerEvent | null) {
        super.pointerMove(pointer);
        this.pointerMoved = true;
        this.scene.forceUpdateScene();
    }

    pointerUp(pointer: PointerEvent | null) {
        super.pointerUp(pointer);

        if (this.pointerMoved) {
            this.scene.forceUpdateCode();
            this.pointerMoved = false;
        }
    }
}
