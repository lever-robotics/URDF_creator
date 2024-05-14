import React, { useRef, useEffect, useState, useCallback, useContext } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';
import { URDFGUIContext } from '../URDFContext/URDFGUIContext';
import { LinkTree } from './LinkTree';

function ThreeScene() {
    const mountRef = useRef(null);
    const mouseData = useRef({ previousUpTime: null, currentDownTime: null, startPos: null });


    // State to manage the currently selected object and its position
    const [selectedObject, setSelectedObject] = useState(null);
    const [objectPosition, setObjectPosition] = useState({ x: 0, y: 0, z: 0 });
    const [treeJSON, setTreeJson] = useState({});

    // This ref will hold the Three.js essentials
    const threeObjects = useRef({
        scene: null,
        camera: null,
        renderer: null,
        orbitControls: null,
        transformControls: null,
        ambientLight: null,
        raycaster: new THREE.Raycaster(),
        mouse: new THREE.Vector2(),
        initialized: false
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

        // Add a grid helper to the scene
        const gridHelper = new THREE.GridHelper(10, 10);
        gridHelper.userData.selectable = false;
        obj.scene.add(gridHelper);

        function selectObject(event) {
            obj.mouse.x = (event.clientX / mountRef.current.clientWidth) * 2 - 1;
            obj.mouse.y = -(event.clientY / mountRef.current.clientHeight) * 2 + 1;
            obj.raycaster.setFromCamera(obj.mouse, obj.camera);
            const intersects = obj.raycaster.intersectObjects(obj.scene.children, false);
            const meshes = intersects.filter((collision) => collision.object instanceof THREE.Mesh)
            if (meshes.length > 0) {
                const firstIntersectedObject = meshes[0].object;
                if (firstIntersectedObject.userData.selectable !== false) {
                    setSelectedObject(firstIntersectedObject);
                    obj.transformControls.attach(firstIntersectedObject);
                    setObjectPosition(firstIntersectedObject.position);
                } else {
                    setSelectedObject(null);
                    obj.transformControls.detach();
                }
            } else {
                setSelectedObject(null);
                obj.transformControls.detach();
            }
        }



        function onDoubleClick(event) {

        }

        function onClick(event) {
            selectObject(event)
        }

        function onMouseUp(event) {
            event.preventDefault();
            // ms between clicks for a click to be registered
            const clickTime = 300;
            const dragThreshold = 5;
            const endPos = [event.clientX, event.clientY];

            // if the mouse is dragged, do nothing
            if (Math.sqrt((endPos[0] - mouseData.current.startPos[0]) ** 2 + (endPos[1] - mouseData.current.startPos[1]) ** 2) > dragThreshold) { }
            // if the mouse is double clicked, call double click
            else if (mouseData.current.currentDownTime - mouseData.current.previousUpTime < clickTime && Date.now() - mouseData.current.currentDownTime < clickTime) {
                onDoubleClick(event)
            }
            // if the mouse is single clicked, call click
            else if (Date.now() - mouseData.current.currentDownTime < clickTime) {
                onClick(event)

            }
            mouseData.current.previousUpTime = Date.now()
        }

        // Event listener for mouse down to select objects
        function onMouseDown(event) {
            event.preventDefault();
            mouseData.current.currentDownTime = Date.now()
            mouseData.current.startPos = [event.clientX, event.clientY]
        }

        mountRef.current.addEventListener('pointerdown', onMouseDown);
        mountRef.current.addEventListener('pointerup', onMouseUp);

        obj.initialized = true;

        const animate = () => {
            requestAnimationFrame(animate);
            obj.renderer.render(obj.scene, obj.camera);
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
        console.log('Setting the current Scene to be local variable');
        console.log('Current Scene is: ', threeObjects.current.scene);
        // Update the current scene in the context
        setCurrentScene(threeObjects.current.scene);

        console.log('Doing a save to global state');
        // Now call the saveURDFTree to handle the saving logic
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
        obj.scene.add(mesh);
        setTreeJson(obj.scene.toJSON())
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
        <div>
            <div className='row-no-space'>
                {/* The main threejs display */}
                <div ref={mountRef} style={{ width: '800px', height: '600px' }} />
                {/* Tree structure menu */}
                <LinkTree tree={treeJSON}></LinkTree>
            </div>

            <div style={{ marginTop: '10px' }}>
                <button onClick={() => addObject("cube")}>Add Cube</button>
                <button onClick={() => addObject("sphere")}>Add Sphere</button>
                <button onClick={() => addObject("cylinder")}>Add Cylinder</button>
            </div>
            <div style={{ marginTop: '10px' }}>
                <button onClick={() => setTransformMode("translate")}>Translate</button>
                <button onClick={() => setTransformMode("rotate")}>Rotate</button>
                <button onClick={() => setTransformMode("scale")}>Scale</button>
            </div>
            {selectedObject && (
                <div style={{ marginTop: '10px', padding: '10px', border: '1px solid #ddd' }}>
                    <h3>Object Info</h3>
                    <div>
                        <label>X: </label>
                        <input
                            type="number"
                            value={objectPosition.x.toFixed(2)}
                            onChange={(e) => handlePositionChange('x', e.target.value)}
                        />
                    </div>
                    <div>
                        <label>Y: </label>
                        <input
                            type="number"
                            value={objectPosition.y.toFixed(2)}
                            onChange={(e) => handlePositionChange('y', e.target.value)}
                        />
                    </div>
                    <div>
                        <label>Z: </label>
                        <input
                            type="number"
                            value={objectPosition.z.toFixed(2)}
                            onChange={(e) => handlePositionChange('z', e.target.value)}
                        />
                    </div>
                </div>
            )}
            {/* Add the Save Button */}
            <div style={{ marginTop: '10px' }}>
                <button onClick={handleSave}>Save</button>
            </div>
        </div>
    );
}

export default ThreeScene;
