import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "../../Models/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./Mouse";
import ThreeScene from "./ThreeSceneObject";

//For putting letters in the scene
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import { RefObject } from "react";
import { StateFunctionsType } from "../SceneState";


export class ThreeSceneManager {
    constructScene(mountRef: React.MutableRefObject<HTMLDivElement | null>, stateFunctions: StateFunctionsType) {
        const scene = this.setupScene();
        const camera = this.setupCamera(mountRef.current!);
        const renderer = this.setupRenderer(mountRef.current!);
        const orbitControls = this.setupOrbitControls(camera, renderer);
        const transformControls = this.setupTransformControls(camera, renderer, stateFunctions);
        const lights: THREE.Light[] = this.setupLights();
        const gridHelper = this.setupGridHelper();
        const axesHelper = this.setupAxesHelper();
        const renderPass = this.setupRenderPass(scene, camera);
        const composer = this.setupComposer(renderer, renderPass);
        const background = this.setupBackground();
        const raycaster = this.setupRaycaster();
        const mouse = this.setupMouse(mountRef);
        const callback = this.setupCallback(orbitControls, transformControls, renderer, scene, mountRef);

        const three = new ThreeScene(
            mountRef,
            scene,
            camera,
            renderer,
            orbitControls,
            transformControls,
            lights,
            gridHelper,
            axesHelper,
            renderPass,
            composer,
            background,
            raycaster,
            mouse,
            callback
        );

        three.addToScene([three.transformControls, three.lights[0], three.lights[1], three.lights[2], three.lights[3], three.gridHelper, three.axesHelper]);

        three.transformControls.addEventListener("dragging-changed", (event: any) => {
            three.orbitControls.enabled = !event.value;
        });

        three.scene.background = three.background;

        return three;
    }

    setupScene() {
        return new THREE.Scene();
    }

    setupCamera(mount: HTMLDivElement) {
        const camera = new THREE.PerspectiveCamera(75, mount.clientWidth / mount.clientHeight, 0.1, 1000);
        camera.position.set(5, 5, 5);
        camera.up.set(0, 0, 1);
        return camera;
    }

    setupRenderer(mount: HTMLElement) {
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(mount.clientWidth, mount.clientHeight);
        mount.appendChild(renderer.domElement);
        return renderer;
    }

    setupOrbitControls(camera: THREE.Camera, renderer: THREE.WebGLRenderer) {
        return new OrbitControls(camera, renderer.domElement);
    }

    setupTransformControls(camera: THREE.Camera, renderer: THREE.WebGLRenderer, stateFunctions: any) {
        return new TransformControls(camera, renderer.domElement, stateFunctions);
    }

    setupLights() {
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        const directionalLight1 = new THREE.DirectionalLight(0xffffff, 1);
        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 1);
        const pointLight = new THREE.PointLight(0xffffff, 0.5);
        directionalLight1.position.set(5, 10, 7.5);
        directionalLight2.position.set(-5, -10, -7.5);
        pointLight.position.set(0, 5, 0);
        return [ambientLight, directionalLight1, directionalLight2, pointLight];
    }

    setupGridHelper() {
        const gridHelper = new THREE.GridHelper(10, 10);
        gridHelper.rotation.x = Math.PI / 2;
        return gridHelper;
    }

    setupFont(scene: THREE.Scene, camera: THREE.Camera) {
        const fontLoader = new FontLoader();
        fontLoader.load("/fonts/helvetiker_regular.typeface.json", (font) => {
            const textMaterial = new THREE.MeshBasicMaterial({
                color: 0xffffff,
            }); // Change color to blue

            type Position = [number, number, number];

            const textGemetry = (title: string, position: Position) => {
                const textGeo = new TextGeometry(title, {
                    font: font,
                    size: 0.1, // Make the text smaller
                    depth: 0.02,
                    curveSegments: 12,
                    bevelEnabled: false,
                });
                const textMesh = new THREE.Mesh(textGeo, textMaterial);
                textMesh.up.copy(new THREE.Vector3(0, 0, 1));
                textMesh.position.set(...position);
                textMesh.onBeforeRender = () => {
                    textMesh.lookAt(camera.position);
                };
                return textMesh;
            };

            const textMeshX = textGemetry("X", [5, 0, 0]);
            const textMeshY = textGemetry("Y", [0, 5, 0]);
            const textMeshZ = textGemetry("Z", [0, 0, 5]);

            scene.add(textMeshX);
            scene.add(textMeshY);
            scene.add(textMeshZ);

            // Make text always face the camera
            // obj.updateTextRotation = () => {
            //     textMeshX.lookAt(camera.position);
            //     textMeshY.lookAt(camera.position);
            //     textMeshZ.lookAt(camera.position);
            // };
        });
    }

    setupAxesHelper() {
        const axesHelper = new THREE.AxesHelper(50);
        return axesHelper;
    }

    setupRenderPass(scene: THREE.Scene, camera: THREE.Camera) {
        return new RenderPass(scene, camera);
    }

    setupComposer(renderer: THREE.WebGLRenderer, renderPass: RenderPass) {
        const composer = new EffectComposer(renderer);
        composer.addPass(renderPass);
        return composer;
    }

    setupBackground() {
        return new THREE.TextureLoader().load("../../textures/blue.png");
    }

    setupRaycaster() {
        return new THREE.Raycaster();
    }

    setupMouse(mountRef: React.MutableRefObject<HTMLDivElement | null>) {
        const mouse = new Mouse(mountRef);
        mouse.addListeners();
        return mouse;
    }

    setupCallback(orbitControls: OrbitControls, transformControls: TransformControls, renderer: THREE.WebGLRenderer, scene: THREE.Scene, mountRef: RefObject<HTMLElement>) {
        return () => {
            orbitControls.dispose();
            transformControls.dispose();
            renderer.dispose();
            scene.clear();
            if (mountRef.current) {
                mountRef.current.removeChild(renderer.domElement);
            }
        };
    }
}
