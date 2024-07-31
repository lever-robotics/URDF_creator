import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { Mouse } from "./Mouse";
import { ThreeScene } from "./ThreeSceneObject";

//For putting letters in the scene
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";

export class ThreeSceneManager {

    constructScene(mountRef) {
        const scene = this.setupScene();
        const camera = this.setupCamera(mountRef.current);
        const renderer = this.setupRenderer(mountRef.current);
        const orbitControls = this.setupOrbitControls(camera, renderer);
        const transformControls = this.setupTransformControls(camera, renderer);
        const lights = this.setupLights();
        const gridHelper = this.setupGridHelper();
        const font = this.setupFont(scene, camera);
        const axesHelper = this.setupAxesHelper();
        const renderPass = this.setupRenderPass(scene, camera);
        const composer = this.setupComposer(renderer, renderPass);
        const background = this.setupBackground();
        const raycaster = this.setupRaycaster();
        const mouse = this.setupMouse(mountRef.current);
        const callback = this.setupCallback(
            orbitControls,
            transformControls,
            renderer,
            scene,
            mountRef
        );

        const three = new ThreeScene(
            mountRef,
            scene,
            camera,
            renderer,
            orbitControls,
            transformControls,
            lights,
            gridHelper,
            font,
            axesHelper,
            renderPass,
            composer,
            background,
            raycaster,
            mouse,
            callback
        );

        three.addToScene([
            three.transformControls,
            three.lights[0],
            three.lights[1],
            three.lights[2],
            three.lights[3],
            three.gridHelper,
            three.axesHelper,
        ]);

        three.transformControls.addEventListener(
            "dragging-changed",
            (event) => {
                three.orbitControls.enabled = !event.value;
            }
        );

        three.scene.background = three.background;

        return three;
    }

    setupScene() {
        return new THREE.Scene();
    }

    setupCamera(mountRef) {
        const camera = new THREE.PerspectiveCamera(
            75,
            mountRef.clientWidth / mountRef.clientHeight,
            0.1,
            1000
        );
        camera.position.set(5, 5, 5);
        camera.up.set(0, 0, 1);
        return camera;
    }

    setupRenderer(mountRef) {
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(mountRef.clientWidth, mountRef.clientHeight);
        mountRef.appendChild(renderer.domElement);
        return renderer;
    }

    setupOrbitControls(camera, renderer) {
        return new OrbitControls(camera, renderer.domElement);
    }

    setupTransformControls(camera, renderer) {
        return new TransformControls(camera, renderer.domElement);
    }

    setupLights(color) {
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

    setupFont(scene, camera) {
        const fontLoader = new FontLoader();
        fontLoader.load(
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
            }
        );
    }

    setupAxesHelper() {
        const axesHelper = new THREE.AxesHelper(50);
        return axesHelper;
    }

    setupRenderPass(scene, camera) {
        return new RenderPass(scene, camera);
    }

    setupComposer(renderer, renderPass) {
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

    setupMouse(mountRef) {
        const mouse = new Mouse(mountRef);
        mouse.addListeners();
        return mouse;
    }

    setupCallback(orbitControls, transformControls, renderer, scene, mountRef) {
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
