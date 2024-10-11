import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./Mouse";
import ThreeScene from "./ThreeScene";

import type { RefObject } from "react";
import { Vector3 } from "three";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
//For putting letters in the scene
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";
import TransformControls from "../../Models/TransformControls";

export function constructThreeScene(mountDiv: HTMLDivElement) {
    const camera = setupCamera(mountDiv);
    const renderer = setupRenderer(mountDiv);
    const orbitControls = setupOrbitControls(camera, renderer);
    const transformControls = setupTransformControls(camera, renderer);
    const lights: THREE.Light[] = setupLights();
    const gridHelper = setupGridHelper();
    const axesHelper = setupAxesHelper();
    const scene = setupScene([
        transformControls,
        lights[0],
        lights[1],
        lights[2],
        lights[3],
        gridHelper,
        axesHelper,
    ]);
    const renderPass = setupRenderPass(scene, camera);
    const composer = setupComposer(renderer, renderPass);
    const background = setupBackground();
    const raycaster = setupRaycaster();
    const mouse = setupMouse(mountDiv);
    const callback = setupCallback(
        orbitControls,
        transformControls,
        renderer,
        scene,
        mountDiv,
    );

    const font = setupFont(scene, camera);

    scene.background = background;

    const three = new ThreeScene(
        mountDiv,
        scene,
        camera,
        orbitControls,
        transformControls,
        composer,
        raycaster,
        mouse,
        callback,
    );

    transformControls.scene = three;

    three.transformControls.addEventListener(
        "dragging-changed",
        (event: { value: unknown }) => {
            three.orbitControls.enabled = !event.value;
        },
    );

    return three;
}

function setupScene(objects: THREE.Object3D[]) {
    const scene = new THREE.Scene();
    for (const object of objects) {
        scene.add(object);
    }
    return scene;
}

function setupCamera(mount: HTMLDivElement) {
    const camera = new THREE.PerspectiveCamera(
        75,
        mount.clientWidth / mount.clientHeight,
        0.1,
        1000,
    );
    camera.position.set(5, 5, 5);
    camera.up.set(0, 0, 1);
    return camera;
}

function setupRenderer(mount: HTMLElement) {
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(mount.clientWidth, mount.clientHeight);
    mount.appendChild(renderer.domElement);
    return renderer;
}

function setupOrbitControls(
    camera: THREE.Camera,
    renderer: THREE.WebGLRenderer,
) {
    return new OrbitControls(camera, renderer.domElement);
}

function setupTransformControls(
    camera: THREE.Camera,
    renderer: THREE.WebGLRenderer,
) {
    return new TransformControls(camera, renderer.domElement);
}

function setupLights() {
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    const directionalLight1 = new THREE.DirectionalLight(0xffffff, 1);
    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 1);
    const pointLight = new THREE.PointLight(0xffffff, 0.5);
    directionalLight1.position.set(5, 10, 7.5);
    directionalLight2.position.set(-5, -10, -7.5);
    pointLight.position.set(0, 5, 0);
    return [ambientLight, directionalLight1, directionalLight2, pointLight];
}

function setupGridHelper() {
    const gridHelper = new THREE.GridHelper(10, 10);
    gridHelper.rotation.x = Math.PI / 2;
    return gridHelper;
}

function setupFont(scene: THREE.Scene, camera: THREE.Camera) {
    const fontLoader = new FontLoader();
    fontLoader.load("/fonts/helvetiker_regular.typeface.json", (font) => {
        const textMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,
        }); // Change color to blue

        const textGemetry = (title: string, position: Vector3) => {
            const textGeo = new TextGeometry(title, {
                font: font,
                size: 0.1, // Make the text smaller
                depth: 0.02,
                curveSegments: 12,
                bevelEnabled: false,
            });
            const textMesh = new THREE.Mesh(textGeo, textMaterial);
            textMesh.up.copy(new THREE.Vector3(0, 0, 1));
            textMesh.position.copy(position);
            textMesh.onBeforeRender = () => {
                textMesh.lookAt(camera.position);
            };
            return textMesh;
        };

        const textMeshX = textGemetry("X", new Vector3(5, 0, 0));
        const textMeshY = textGemetry("Y", new Vector3(0, 5, 0));
        const textMeshZ = textGemetry("Z", new Vector3(0, 0, 5));

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

function setupAxesHelper() {
    const axesHelper = new THREE.AxesHelper(50);
    return axesHelper;
}

function setupRenderPass(scene: THREE.Scene, camera: THREE.Camera) {
    return new RenderPass(scene, camera);
}

function setupComposer(renderer: THREE.WebGLRenderer, renderPass: RenderPass) {
    const composer = new EffectComposer(renderer);
    composer.addPass(renderPass);
    return composer;
}

function setupBackground() {
    return new THREE.TextureLoader().load("../../textures/blue.png");
}

function setupRaycaster() {
    return new THREE.Raycaster();
}

function setupMouse(mountDiv: HTMLDivElement) {
    const mouse = new Mouse(mountDiv);
    mouse.addListeners();
    return mouse;
}

function setupCallback(
    orbitControls: OrbitControls,
    transformControls: TransformControls,
    renderer: THREE.WebGLRenderer,
    scene: THREE.Scene,
    mountDiv: HTMLElement,
) {
    return () => {
        orbitControls.dispose();
        transformControls.dispose();
        renderer.dispose();
        scene.clear();
        mountDiv.removeChild(renderer.domElement);
    };
}
