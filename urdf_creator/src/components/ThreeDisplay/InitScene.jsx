import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { TransformControls } from "three/examples/jsm/controls/TransformControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";

//For putting letters in the scene
import { FontLoader } from 'three/examples/jsm/loaders/FontLoader';
import { TextGeometry } from 'three/examples/jsm/geometries/TextGeometry';

export default function initScene(threeObjects, mountRef) {
    const { current: obj } = threeObjects;
    if (!mountRef.current || obj.initialized) return;

    // Initialize the scene, camera, and renderer
    obj.scene = new THREE.Scene();
    obj.camera = new THREE.PerspectiveCamera(75, mountRef.current.clientWidth / mountRef.current.clientHeight, 0.1, 1000);
    obj.camera.position.set(5, 5, 5);
    obj.camera.up.set(0, 0, 1);

    obj.renderer = new THREE.WebGLRenderer({ antialias: true });
    obj.renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(obj.renderer.domElement);

    // Initialize and configure OrbitControls
    obj.orbitControls = new OrbitControls(obj.camera, obj.renderer.domElement);

    // Initialize and configure TransformControls
    obj.transformControls = new TransformControls(obj.camera, obj.renderer.domElement);
    obj.scene.add(obj.transformControls);

    // Disable orbit controls when transforming objects
    obj.transformControls.addEventListener("dragging-changed", (event) => {
        obj.orbitControls.enabled = !event.value;
    });

    // Add an ambient light to the scene
    obj.ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    obj.scene.add(obj.ambientLight);

    // Add directional lights to the scene
    obj.directionalLight1 = new THREE.DirectionalLight(0xffffff, 1);
    obj.directionalLight1.position.set(5, 10, 7.5);
    obj.scene.add(obj.directionalLight1);

    obj.directionalLight2 = new THREE.DirectionalLight(0xffffff, 1);
    obj.directionalLight2.position.set(-5, -10, -7.5);
    obj.scene.add(obj.directionalLight2);

    // Add a point light to the scene
    obj.pointLight = new THREE.PointLight(0xffffff, 0.5);
    obj.pointLight.position.set(0, 5, 0);
    obj.scene.add(obj.pointLight);

    // Add a grid helper to the scene
    const gridHelper = new THREE.GridHelper(10, 10);
    gridHelper.userData.selectable = false;
    gridHelper.rotation.x = Math.PI / 2; // Rotate the grid to lie on the XY plane
    obj.scene.add(gridHelper);

    // Load font and create text meshes
    const loader = new FontLoader();
    loader.load(process.env.PUBLIC_URL + '/fonts/helvetiker_regular.typeface.json', function (font) {
        const textMaterial = new THREE.MeshBasicMaterial({ color: 0xFFFFFF }); // Change color to blue

        // Create the text geometry for X
        const textGeometryX = new TextGeometry('X', {
            font: font,
            size: 0.1, // Make the text smaller
            height: 0.02,
            curveSegments: 12,
            bevelEnabled: false,
        });
        const textMeshX = new THREE.Mesh(textGeometryX, textMaterial);
        textMeshX.up.copy(new THREE.Vector3(0, 0, 1))
        textMeshX.onBeforeRender = () => {
            textMeshX.lookAt(obj.camera.position)
        }
        textMeshX.position.set(5, 0, 0);
        obj.scene.add(textMeshX);

        // Create the text geometry for Y
        const textGeometryY = new TextGeometry('Y', {
            font: font,
            size: 0.1, // Make the text smaller
            height: 0.02,
            curveSegments: 12,
            bevelEnabled: false,
        });
        const textMeshY = new THREE.Mesh(textGeometryY, textMaterial);
        textMeshY.up.copy(new THREE.Vector3(0, 0, 1))
        textMeshY.onBeforeRender = () => {
            textMeshY.lookAt(obj.camera.position)
        }
        textMeshY.position.set(0, 5, 0);
        obj.scene.add(textMeshY);

        // Create the text geometry for Z
        const textGeometryZ = new TextGeometry('Z', {
            font: font,
            size: 0.1, // Make the text smaller
            height: 0.02,
            curveSegments: 12,
            bevelEnabled: false,
        });
        const textMeshZ = new THREE.Mesh(textGeometryZ, textMaterial);
        textMeshZ.up.copy(new THREE.Vector3(0, 0, 1))
        textMeshZ.onBeforeRender = () => {
            textMeshZ.lookAt(obj.camera.position)
        }
        textMeshZ.position.set(0, 0, 5);
        obj.scene.add(textMeshZ);

        // Make text always face the camera
        obj.updateTextRotation = () => {
            textMeshX.lookAt(obj.camera.position);
            textMeshY.lookAt(obj.camera.position);
            textMeshZ.lookAt(obj.camera.position);
        };
    });

    // Add an axes helper
    const axesHelper = new THREE.AxesHelper(50);  // Length of the axes
    obj.scene.add(axesHelper);

    // Setup Effect Composer for post-processing
    obj.composer = new EffectComposer(obj.renderer);
    const renderPass = new RenderPass(obj.scene, obj.camera);
    obj.composer.addPass(renderPass);

    // Load and apply background gradient
    const background = new THREE.TextureLoader().load("../../textures/blue.png");
    obj.scene.background = background;

    obj.initialized = true;

    const callback = () => {
        obj.orbitControls.dispose();
        obj.transformControls.dispose();
        obj.renderer.dispose();
        obj.scene.clear();
        if (mountRef.current) {
            mountRef.current.removeChild(obj.renderer.domElement);
        }
        obj.initialized = false;
    };

    return callback;
}
