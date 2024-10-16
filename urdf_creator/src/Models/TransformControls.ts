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
        super.pointerDown(pointer);
        this.scene.mountDiv.focus();
    }

    pointerMove(pointer: PointerEvent | null) {
        super.pointerMove(pointer);
        this.pointerMoved = true;
        if (this.scene.selectedObject) this.scene.dispatchEvent("parameters");
    }

    pointerUp(pointer: PointerEvent | null) {
        super.pointerUp(pointer);

        if (this.scene.linkDetached) this.scene.reattachLink();

        if (this.pointerMoved) {
            this.scene.forceUpdateCode();
            this.pointerMoved = false;
        }
    }
}
