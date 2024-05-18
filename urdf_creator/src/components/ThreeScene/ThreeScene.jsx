import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import { useStateContext } from "../URDFContext/StateContext.js";
import { LinkTree } from "./LinkTree";
import initScene from "./InitScene.jsx";
import setUpMouse from "./SetUpMouse.jsx";

function ThreeScene() {
    // the main state of the project
    const { state, dispatch } = useStateContext();

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
    const mountRef = useRef(null);
    const mouseData = useRef({
        previousUpTime: null,
        currentDownTime: null,
        startPos: null,
    });

    useEffect(() => {
        const { current: obj } = threeObjects;
        if (!mountRef.current || obj.initialized) return;

        // perform necessary set up for the threejs scene-- initScene must be called last
        const setUpMouseCallback = setUpMouse(
            threeObjects,
            mountRef,
            mouseData,
            setSelectedObject,
            setObjectPosition,
            setSelectObjectFunc
        );
        const sceneCallback = initScene(threeObjects, mountRef);

        // main animation loop
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
                }
            };

            obj.transformControls.addEventListener(
                "objectChange",
                updatePosition
            );

            return () => {
                obj.transformControls.removeEventListener(
                    "objectChange",
                    updatePosition
                );
            };
        }
    }, [selectedObject]);

    const addObject = (shape) => {
        const { current: obj } = threeObjects;
        if (!obj.scene) return;

        let geometry;
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

        let material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });

        const mesh = new THREE.Mesh(geometry, material);
        mesh.userData.selectable = true;
        mesh.userData.shape = shape;
        mesh.position.set(2.5, 0.5, 2.5);
        if (selectedObject !== null) {
            selectedObject.attach(mesh);
        } else if (baseLink !== null) {
            baseLink.attach(mesh);
        } else {
            mesh.position.set(0, 0.5, 0);
            setBaseLink(mesh);
            obj.scene.attach(mesh);
        }
        setTreeState({ ...obj.scene });
        dispatch({ type: "SET_SCENE", payload: obj.scene });
    };

    const setTransformMode = (mode) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }
    };

    return (
        <div className="row-no-space">
            <div className="left-panel">
                {/* Tree structure menu */}
                <LinkTree tree={treeState} select={selectObjectFunc}></LinkTree>

                <div style={{ marginTop: "10px" }} className="column-box">
                    Add Objects
                    <button onClick={() => addObject("cube")}>Add Cube</button>
                    <button onClick={() => addObject("sphere")}>
                        Add Sphere
                    </button>
                    <button onClick={() => addObject("cylinder")}>
                        Add Cylinder
                    </button>
                </div>
            </div>

            {/* The main threejs display */}
            <div className="display">
                <div ref={mountRef} style={{ width: "100%", height: "100%" }} />
                <div
                    style={{ marginTop: "10px" }}
                    className="row-space-between"
                >
                    <div className="row-spaced">
                        <button onClick={() => setTransformMode("translate")}>
                            Translate
                        </button>
                        <button onClick={() => setTransformMode("rotate")}>
                            Rotate
                        </button>
                        <button onClick={() => setTransformMode("scale")}>
                            Scale
                        </button>
                    </div>
                </div>
            </div>
        </div>
    );
}

export default ThreeScene;
