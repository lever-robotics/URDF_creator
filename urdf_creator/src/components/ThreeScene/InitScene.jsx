import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass';

export default function initScene (threeObjects, mountRef) {
    const { current: obj } = threeObjects;
    if (!mountRef.current || obj.initialized) return;

    // Initialize the scene, camera, and renderer
    obj.scene = new THREE.Scene();
    obj.camera = new THREE.PerspectiveCamera(75, mountRef.current.clientWidth / mountRef.current.clientHeight, 0.1, 1000);
    obj.camera.position.set(5, 5, 5);

    obj.renderer = new THREE.WebGLRenderer({ antialias: true });
    obj.renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
    mountRef.current.appendChild(obj.renderer.domElement);

    // Initialize and configure OrbitControls
    obj.orbitControls = new OrbitControls(obj.camera, obj.renderer.domElement);

    // Initialize and configure TransformControls
    obj.transformControls = new TransformControls(obj.camera, obj.renderer.domElement);
    obj.scene.add(obj.transformControls);

    // Disable orbit controls when transforming objects
    obj.transformControls.addEventListener('dragging-changed', event => {
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
    obj.scene.add(gridHelper);

    // Setup Effect Composer for post-processing
    obj.composer = new EffectComposer(obj.renderer);
    console.log("composer:")
    console.log(obj.composer)
    const renderPass = new RenderPass(obj.scene, obj.camera);
    obj.composer.addPass(renderPass);

    // Load and apply background gradient
    const background = new THREE.TextureLoader().load( "../../textures/blue.png" );
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
    }

    return callback;
}