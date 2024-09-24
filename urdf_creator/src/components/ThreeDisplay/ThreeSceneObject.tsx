import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "../../Models/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./Mouse";
import Frame, { Frameish } from "../../Models/Frame";

export default class ThreeScene {
    rootFrame: Frameish;
    constructor(
        public mountRef: React.RefObject<HTMLElement>,
        public scene: THREE.Scene,
        public camera: THREE.Camera,
        public renderer: THREE.WebGLRenderer,
        public orbitControls: OrbitControls,
        public transformControls: TransformControls,
        public lights: THREE.Light[],
        public gridHelper: THREE.GridHelper,
        public axesHelper: THREE.AxesHelper,
        public renderPass: RenderPass,
        public composer: EffectComposer,
        public background: THREE.Texture,
        public raycaster: THREE.Raycaster,
        public mouse: Mouse,
        public callback: () => void
    ) {
        this.rootFrame = null;
    }
    addToScene(objects: THREE.Object3D[]) {
        objects.forEach((object) => {
            this.scene.add(object);
        });
    }
}
