import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import { useStateContext } from "../URDFContext/StateContext.js";
import { LinkTree } from "./LinkTree";
import initScene from "./InitScene.jsx";
import setUpMouse from "./SetUpMouse.jsx";
import InsertTool from "./InsertTool";
import ObjectParameters from "./ObjectParameters/ObjectParameters.jsx";
import calculateMomentOfInertia from '../../utils/calculateMomentOfInertia'; // Adjust the path as needed

function ThreeScene() {
    // the main state of the project
    const { state, dispatch } = useStateContext();

    // State to manage the currently selected object and its position
    const [selectedObject, setSelectedObject] = useState(null);
    const [baseLink, setBaseLink] = useState(null);
    const [objectPosition, setObjectPosition] = useState({ x: 0, y: 0, z: 0 });
    const [treeState, setTreeState] = useState({});
    const [selectObjectFunc, setSelectObjectFunc] = useState(null);

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
    const mountRef = useRef(null);
    const mouseData = useRef({
        previousUpTime: null,
        currentDownTime: null,
        startPos: null,
    });

    useEffect(() => {
        const { current: obj } = threeObjects;
        if (!mountRef.current || obj.initialized) return;

        const setUpMouseCallback = setUpMouse(threeObjects, mountRef, mouseData, setSelectedObject, setObjectPosition, setSelectObjectFunc);
        const sceneCallback = initScene(threeObjects, mountRef);

        const animate = () => {
            requestAnimationFrame(animate);
            obj.composer.render();
            obj.orbitControls.update();
        };

        animate();
        dispatch({ type: "SET_SCENE", payload: obj.scene });

        return () => {
            sceneCallback();
            setUpMouseCallback();
        };
    }, [dispatch]);

    useEffect(() => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            const updatePosition = () => {
                if (selectedObject) {

                    setObjectPosition({ ...selectedObject.position });

                    // Calculate the moment of inertia and update userData
                    if (!selectedObject.userData.customInertia) {
                        const inertia = calculateMomentOfInertia(selectedObject);
                        selectedObject.userData.Ixx = inertia.Ixx;
                        selectedObject.userData.Ixy = inertia.Ixy;
                        selectedObject.userData.Ixz = inertia.Ixz;
                        selectedObject.userData.Iyy = inertia.Iyy;
                        selectedObject.userData.Iyz = inertia.Iyz;
                        selectedObject.userData.Izz = inertia.Izz;
                    }

                    dispatch({ type: "SET_SCENE", payload: obj.scene });
                }
            };

            obj.transformControls.addEventListener("objectChange", updatePosition);
            return () => {
                obj.transformControls.removeEventListener("objectChange", updatePosition);
            };
        }
    }, [selectedObject]);

    const addObject = (shape) => {
        const { current: obj } = threeObjects;
        if (!obj.scene) return;

        let geometry;
        let onBeforeRender = function (renderer, scene, camera, geometry, material, group) {
            this.children[0].doScale();
        };
        switch (shape) {
            case "cube":
                geometry = new THREE.BoxGeometry(1, 1, 1);
                break;
            case "sphere":
                geometry = new THREE.SphereGeometry(0.5, 32, 32);
                // ensure spheres scale uniformly in all directions
                onBeforeRender = function (renderer, scene, camera, geometry, material, group) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale = (worldScale.x + worldScale.y + worldScale.z) / 3;

                    const localScale = this.scale;
                    this.scale.set((localScale.x / worldScale.x) * uniformScale, (localScale.y / worldScale.y) * uniformScale, (localScale.z / worldScale.z) * uniformScale);
                    this.children[0].doScale();
                };
                break;
            case "cylinder":
                geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                // ensure cylinders scale uniformly in two directions
                onBeforeRender = function (renderer, scene, camera, geometry, material, group) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale = (worldScale.x + worldScale.z) / 2;

                    const localScale = this.scale;
                    this.scale.set((localScale.x / worldScale.x) * uniformScale, localScale.y, (localScale.z / worldScale.z) * uniformScale);
                    this.children[0].doScale();
                };
                break;
            default:
                return;
        }

        let material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });

        const mesh = new THREE.Mesh(geometry, material);
        mesh.onBeforeRender = onBeforeRender;
        mesh.userData = {
            customInertia: false,
            Ixx: 0,
            Ixy: 0,
            Ixz: 0,
            Iyy: 0,
            Iyz: 0,
            Izz: 0,
            isBaseLink: false,
            isSensor: false,
            mass: 1,
            name: shape,
            scaler: new THREE.Object3D(),
            selectable: true,
            sensorType: "",
            shape: shape,
            // IMU Parameters
            gaussianNoise: 0.01,
            xyzOffsets: '0 0 0',
            rpyOffsets: '0 0 0',
            alwaysOn: true,
            updateRate: 100,
            mean: 0,
            stddev: 0,
            // Camera Parameters
            cameraName: 'camera',
            imageTopicName: '/camera/image_raw',
            cameraInfoTopicName: '/camera/camera_info',
            horizontal_fov: 1.3962634,
            width: 800,
            height: 600,
            format: 'R8G8B8',
            near: 0.1,
            far: 100
            // Add additional sensor parameters here
        };

        mesh.position.set(2.5, 0.5, 2.5);

        //uniform scaler stuff so everything doesn't break when you scale and rotate :)
        const scaler = new UniformScaler();
        mesh.userData.scaler = scaler;
        mesh.add(scaler);

        if (selectedObject !== null) {
            selectedObject.userData.scaler.attach(mesh);
        } else if (baseLink !== null) {
            baseLink.userData.scaler.attach(mesh);
        } else {
            mesh.position.set(0, 0.5, 0);
            mesh.userData.isBaseLink = true;
            mesh.userData.name = "base_link";
            setBaseLink(mesh);
            obj.scene.attach(mesh);
        }
        setTreeState({ ...obj.scene });
        dispatch({ type: "SET_SCENE", payload: obj.scene });
    };

    const addObjectToScene = (object) => {
        const { current: obj } = threeObjects;
        object.position.set((Math.random() - 0.5) * 10, 0.5, (Math.random() - 0.5) * 10);
        object.userData.selectable = true;
        if (selectedObject !== null) {
            selectedObject.attach(object);
        } else if (baseLink !== null) {
            baseLink.attach(object);
        } else {
            setBaseLink(object);
            obj.scene.add(object);
        }
        updateTree()
        dispatch({ type: "SET_SCENE", payload: obj.scene });
    };

    const updateTree = () => {
        const { current: obj } = threeObjects;
        setTreeState({ ...obj.scene });
    }

    class UniformScaler extends THREE.Object3D {
        constructor() {
            super();
            this.userData.selectable = false;
            this.userData.scaler = true;
            this.doScale = (renderer, scene, camera, geometry, material, group) => {
                const worldScale = new THREE.Vector3();
                this.getWorldScale(worldScale);
                const uniformScale = 1;
                const localScale = this.scale;
                this.scale.set((localScale.x / worldScale.x) * uniformScale, (localScale.y / worldScale.y) * uniformScale, (localScale.z / worldScale.z) * uniformScale);
            };
        }
    }

    const setTransformMode = (mode) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }
    };

    const handleUpdateObject = (updatedObject) => {
        setSelectedObject(updatedObject);
        setTreeState({ ...threeObjects.current.scene });
        dispatch({ type: "SET_SCENE", payload: threeObjects.current.scene });
    };

    return (
        <div className="row-no-space">
            <div className="left-panel">
                <LinkTree tree={treeState} select={selectObjectFunc} updateTree={updateTree} selectedObject={selectedObject} setSelectedObject={setSelectedObject} transformControls={threeObjects.current.transformControls}></LinkTree>
                <InsertTool addObject={addObject} addObjectToScene={addObjectToScene} />
            </div>

            <ObjectParameters selectedObject={selectedObject} onUpdate={handleUpdateObject} />

            <div className="display">
                <div ref={mountRef} style={{ width: "100%", height: "100%" }} />
                <div style={{ marginTop: "10px" }} className="row-space-between">
                    <div className="row-spaced">
                        <button onClick={() => setTransformMode("translate")}>Translate</button>
                        <button onClick={() => setTransformMode("rotate")}>Rotate</button>
                        <button onClick={() => setTransformMode("scale")}>Scale</button>
                    </div>
                </div>
            </div>
        </div>
    );
}

export default ThreeScene;
