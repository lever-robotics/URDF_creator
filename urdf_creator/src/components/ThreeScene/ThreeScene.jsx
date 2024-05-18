import React, { useRef, useEffect, useState, useCallback, useContext } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass';
import { URDFGUIContext } from '../URDFContext/URDFGUIContext';
import { LinkTree } from './LinkTree';

function ThreeScene() {
    const mountRef = useRef(null);
    const mouseData = useRef({ previousUpTime: null, currentDownTime: null, startPos: null });

    // State to manage the currently selected object and its position
    const [selectedObject, setSelectedObject] = useState(null);
    const [baseLink, setBaseLink] = useState(null);
    const [objectPosition, setObjectPosition] = useState({ x: 0, y: 0, z: 0 });
    const [treeState, setTreeState] = useState({});
    const [selectObjectFunc, setSelectObjectFunc] = useState(null);

    // This ref will hold the Three.js essentials
    const threeObjects = useRef({
        scene: null,
        camera: null,
        renderer: null,
        orbitControls: null,
        transformControls: null,
        ambientLight: null,
        directionalLight1: null,
        directionalLight2: null,
        pointLight: null,
        raycaster: new THREE.Raycaster(),
        mouse: new THREE.Vector2(),
        initialized: false,
        composer: null,
        outlinePass: null,
    });

    const { currentScene, updateURDFScene, saveURDFScene, setCurrentScene } = useContext(URDFGUIContext);

    useEffect(() => {
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
        const renderPass = new RenderPass(obj.scene, obj.camera);
        obj.composer.addPass(renderPass);

        // Load and apply background gradient
        const background = new THREE.TextureLoader().load( "../../textures/blue.png" );
        obj.scene.background = background;

        function clickObject(event) {
            const rect = mountRef.current.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;

            obj.mouse.x = (x / rect.width) * 2 - 1;
            obj.mouse.y = -(y / rect.height) * 2 + 1;

            obj.raycaster.setFromCamera(obj.mouse, obj.camera);
            const intersects = obj.raycaster.intersectObjects(obj.scene.children);

            const shapes = intersects.filter((collision) => collision.object.userData.shape);
            const meshes = intersects.filter((collision) => collision.object.type === "Mesh");

            if (shapes.length > 0) {
                const object = shapes[0].object;
                if (object.userData.selectable !== false) {
                    setSelectedObject(object);
                    obj.transformControls.attach(object);
                    setObjectPosition(object.position);
                } else {
                    setSelectedObject(null);
                    obj.transformControls.detach();
                }
            } else if (meshes.length === 0) {
                setSelectedObject(null);
                obj.transformControls.detach();
            }
        }

        setSelectObjectFunc(() => selectObject);

        const selectObject = (object) => {
            if (object.userData.selectable !== false) {
                setSelectedObject(object);
                obj.transformControls.attach(object);
                setObjectPosition(object.position);
            } else {
                setSelectedObject(null);
                obj.transformControls.detach();
            }
        }

        function onDoubleClick(event) {
            // Handle double click event if needed
        }

        function onClick(event) {
            clickObject(event)
        }

        function onMouseUp(event) {
            event.preventDefault();
            const clickTime = 300;
            const dragThreshold = 20;
            const endPos = [event.clientX, event.clientY];

            if (Math.sqrt((endPos[0] - mouseData.current.startPos[0]) ** 2 + (endPos[1] - mouseData.current.startPos[1]) ** 2) > dragThreshold) {
                // Do nothing if dragged
            } else if (mouseData.current.currentDownTime - mouseData.current.previousUpTime < clickTime && Date.now() - mouseData.current.currentDownTime < clickTime) {
                onDoubleClick(event);
            } else if (Date.now() - mouseData.current.currentDownTime < clickTime) {
                onClick(event);
            }
            mouseData.current.previousUpTime = Date.now();
        }

        function onMouseDown(event) {
            event.preventDefault();
            mouseData.current.currentDownTime = Date.now();
            mouseData.current.startPos = [event.clientX, event.clientY];
        }

        mountRef.current.addEventListener('pointerdown', onMouseDown);
        mountRef.current.addEventListener('pointerup', onMouseUp);

        obj.initialized = true;

        const animate = () => {
            requestAnimationFrame(animate);
            obj.composer.render();
            obj.orbitControls.update();
        };
        animate();

        return () => {
            if (mountRef.current) {
                mountRef.current.removeEventListener('pointerdown', onMouseDown);
                mountRef.current.removeEventListener('pointerup', onMouseUp);
            }
            obj.orbitControls.dispose();
            obj.transformControls.dispose();
            obj.renderer.dispose();
            obj.scene.clear();
            if (mountRef.current) {
                mountRef.current.removeChild(obj.renderer.domElement);
            }
            obj.initialized = false;
        };
    }, []);

    const handleSave = useCallback(() => {
        setCurrentScene(threeObjects.current.scene);
        saveURDFScene(threeObjects.current.scene);
    }, [setCurrentScene, saveURDFScene]);

    useEffect(() => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            const updatePosition = () => {
                if (selectedObject) {
                    setObjectPosition({ ...selectedObject.position });
                }
            };

            obj.transformControls.addEventListener('objectChange', updatePosition);

            return () => {
                obj.transformControls.removeEventListener('objectChange', updatePosition);
            };
        }
    }, [selectedObject]);

    const addObject = shape => {
        const { current: obj } = threeObjects;
        if (!obj.scene) return;

        let geometry;
        let material = new THREE.MeshPhongMaterial({ color: Math.random() * 0xffffff });

        switch (shape) {
            case "cube":
                geometry = new THREE.BoxGeometry(1, 1, 1);
                break;
            case "sphere":
                geometry = new THREE.SphereGeometry(0.5, 32, 32);
                break;
            case "cylinder":
                geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                break;
            default:
                return;
        }

        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set((Math.random() - 0.5) * 10, 0.5, (Math.random() - 0.5) * 10);
        mesh.userData.selectable = true;
        mesh.userData.shape = shape;
        if (selectedObject !== null) {
            selectedObject.attach(mesh);
        } else if (baseLink !== null) {
            baseLink.attach(mesh);
        } else {
            setBaseLink(mesh);
            obj.scene.attach(mesh);
        }
        setTreeState({ ...obj.scene });
    };

    const handlePositionChange = (axis, value) => {
        const { current: obj } = threeObjects;
        if (selectedObject && obj.transformControls) {
            selectedObject.position[axis] = Number(value);
            setObjectPosition({ ...objectPosition, [axis]: Number(value) });
            obj.transformControls.update();
        }
    };

    const setTransformMode = (mode) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }
    }

    return (
        <div className='row-no-space'>
            <div className='left-panel'>
                {/* Tree structure menu */}
                <LinkTree tree={treeState} select={selectObjectFunc}></LinkTree>

                <div style={{ marginTop: '10px' }} className='column-box'>
                    Add Objects
                    <button onClick={() => addObject("cube")}>Add Cube</button>
                    <button onClick={() => addObject("sphere")}>Add Sphere</button>
                    <button onClick={() => addObject("cylinder")}>Add Cylinder</button>
                </div>
            </div>
            
            {/* The main threejs display */}
            <div className='display'>
                <div ref={mountRef} style={{ width: '100%', height: '100%' }} />
                <div style={{ marginTop: '10px' }} className='row-space-between'>
                    <div className='row-spaced'>
                        <button onClick={() => setTransformMode("translate")}>Translate</button>
                        <button onClick={() => setTransformMode("rotate")}>Rotate</button>
                        <button onClick={() => setTransformMode("scale")}>Scale</button>
                    </div>
                    <button onClick={handleSave} style={{ backgroundColor: '#7A8F9A' }}>Generate URDF</button>
                </div>
            </div>
        </div>
    );
}

export default ThreeScene;
