import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./SetUpMouse";

//For putting letters in the scene
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";

class ThreeScene {
    constructor(mountRef) {
        this.mountRef = mountRef;
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            75,
            mountRef.current.clientWidth / mountRef.current.clientHeight,
            0.1,
            1000
        );
        this.camera.position.set(5,5,5);
        this.camera.up.set(0,0,1);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.orbitControls = new OrbitControls(
            this.camera,
            this.renderer.domElement
        );
        // this.orbitControls.rotateSpeed = -1;
        // this.orbitControls.panSpeed = -1;
        this.transformControls = new TransformControls(
            this.camera,
            this.renderer.domElement
        );
        this.ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.directionalLight1 = new THREE.DirectionalLight(0xffffff, 1);
        this.directionalLight2 = new THREE.DirectionalLight(0xffffff, 1);
        this.pointLight = new THREE.PointLight(0xffffff, 0.5);
        this.gridHelper = new THREE.GridHelper(10, 10);
        this.fontLoader = new FontLoader();
        this.axesHelper = new THREE.AxesHelper(50);
        this.composer = new EffectComposer(this.renderer);
        this.renderPass = new RenderPass(this.scene, this.camera);
        this.background = new THREE.TextureLoader().load(
            "../../textures/blue.png"
        );
        this.callback = () => {
            this.orbitControls.dispose();
            this.transformControls.dispose();
            this.renderer.dispose();
            this.scene.clear();
            if (mountRef.current) {
                mountRef.current.removeChild(this.renderer.domElement);
            }
            this.initialized = false;
        };
        this.baseLink = null;
        this.raycaster = new THREE.Raycaster();
        this.mouse = new Mouse(this.mountRef);
        this.mouse.addListeners();
    }

    //double back and add everything
    //Add event listeners
    addToScene(objects) {
        objects.forEach((object) => {
            this.scene.add(object);
        });
    }

    setupCamera() {
        this.camera.position.set(5, 5, 5);
        this.camera.up.set(0, 0, 1);
        return this;
    }

    setupRenderer() {
        this.renderer.setSize(
            this.mountRef.current.clientWidth,
            this.mountRef.current.clientHeight
        );
        this.mountRef.current.appendChild(this.renderer.domElement);
        return this;
    }

    setupLights() {
        this.directionalLight1.position.set(5, 10, 7.5);
        this.directionalLight2.position.set(-5, -10, -7.5);
        this.pointLight.position.set(0, 5, 0);
        return this;
    }

    setupGridHelper() {
        this.gridHelper.rotation.x = Math.PI / 2;
        return this;
    }

    setupFont() {
        const obj = this;
        this.fontLoader.load(
            process.env.PUBLIC_URL + "/fonts/helvetiker_regular.typeface.json",
            (font) => {
                const textMaterial = new THREE.MeshBasicMaterial({
                    color: 0xffffff,
                }); // Change color to blue

                const textGemetry = (title, position) => {
                    const textGeo = new TextGeometry(title, {
                        font: font,
                        size: 0.1, // Make the text smaller
                        height: 0.02,
                        curveSegments: 12,
                        bevelEnabled: false,
                    });
                    const textMesh = new THREE.Mesh(textGeo, textMaterial);
                    textMesh.up.copy(new THREE.Vector3(0, 0, 1));
                    textMesh.position.set(...position);
                    textMesh.onBeforeRender = () => {
                        textMesh.lookAt(obj.camera.position);
                    };
                    return textMesh;
                };

                const textMeshX = textGemetry("X", [5, 0, 0]);
                const textMeshY = textGemetry("Y", [0, 5, 0]);
                const textMeshZ = textGemetry("Z", [0, 0, 5]);

                obj.scene.add(textMeshX);
                obj.scene.add(textMeshY);
                obj.scene.add(textMeshZ);

                // Make text always face the camera
                obj.updateTextRotation = () => {
                    textMeshX.lookAt(obj.camera.position);
                    textMeshY.lookAt(obj.camera.position);
                    textMeshZ.lookAt(obj.camera.position);
                };
            }
        );
        return this;
    }

    setupComposer() {
        this.composer.addPass(this.renderPass);
        return this;
    }
}

export default function InitScene(mountRef) {
    const three = new ThreeScene(mountRef);

    three
        .setupCamera()
        .setupRenderer()
        .setupLights()
        .setupGridHelper()
        .setupFont()
        .setupComposer();

    three.addToScene([
        three.transformControls,
        three.ambientLight,
        three.directionalLight1,
        three.directionalLight2,
        three.pointLight,
        three.gridHelper,
        three.axesHelper,
    ]);

    three.transformControls.addEventListener("dragging-changed", (event) => {
        three.orbitControls.enabled = !event.value;
    });

    three.scene.background = three.background;

    three.initialized = true;

    return three;
}
