import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import initScene from "./ThreeDisplay/InitScene.jsx";
import setUpSceneMouse from "./ThreeDisplay/SetUpMouse.jsx";
import UserData from "../Models/UserData.jsx";
import ThreeDisplay from "./ThreeDisplay/ThreeDisplay.jsx";
import ObjectParameters from "./ObjectParameters/ObjectParameters.jsx";
import Toolbar from "./Toolbar/ToolBar.jsx";
import InsertTool from "./Insert/InsertTool.jsx";
import { LinkTree } from "./TreeView/LinkTree.jsx";
import CodeDisplay from "./CodeDisplay/CodeDisplay.jsx";

export default function SceneState() {

    // State to manage the currently selected object and its position
    const [selectedObject, setSelectedObject] = useState(null);
    const [scene, setScene] = useState();
    const [numShapes, setNumShapes] = useState({
        cube: 0,
        sphere: 0,
        cylinder: 0,
    });
    // three objects contain all the objects needed for the threescene
    //This data is only used for threescene
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
        baseLink: null,
    });

    const mountRef = useRef(null);
    const mouseData = useRef({
        previousUpTime: null,
        currentDownTime: null,
        startPos: null,
    });

    console.log("SceneState.jsx")

    // Set up the scene (initialization)
    useEffect(() => {
        const { current: obj } = threeObjects;
        if (!mountRef.current || obj.initialized) return;

        const setUpMouseCallback = setUpSceneMouse(threeObjects, mountRef, mouseData, selectObject);
        const sceneCallback = initScene(threeObjects, mountRef);
        console.log("set up scene")
        console.log(obj.scene)
        setScene({ ...obj.scene });

        const animate = () => {
            requestAnimationFrame(animate);
            obj.composer.render();
            obj.orbitControls.update();
        };

        animate();

        return () => {
            sceneCallback();
            setUpMouseCallback();
        };
    }, []);


    const addObject = (shape) => {
        const { current: obj } = threeObjects;
        if (!obj.scene) return;

        let geometry;
        let onBeforeRender = function (renderer, scene, camera, geometry, material, group) {
            this.userData.scaler.doScale();
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
                    this.userData.scaler.doScale();
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
                    this.userData.scaler.doScale();
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
        mesh.userData = new UserData(shape, shape + (numShapes[shape] + 1).toString());
        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        mesh.position.set(2.5, 0.5, 2.5);

        //uniform scaler stuff so everything doesn't break when you scale and rotate :)

        mesh.add(mesh.userData.scaler);

        if (selectedObject !== null) {
            selectedObject.userData.scaler.attach(mesh);
        } else if (obj.baseLink !== null) {
            obj.baseLink.userData.scaler.attach(mesh);
        } else {
            mesh.position.set(0, 0.5, 0);
            mesh.userData.isBaseLink = true;
            mesh.userData.name = "base_link";
            obj.baseLink = mesh;
            obj.scene.attach(mesh);
        }
        forceSceneUpdate();
    };


    const forceSceneUpdate = () => {
        const { current: obj } = threeObjects;
        setScene({ ...obj.scene });
    }



    const setTransformMode = (mode) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }
    };

    const selectObject = (object) => {
        const { current: obj } = threeObjects;
        if (object.userData.selectable) {
            setSelectedObject(object);
            obj.transformControls.attach(object);
        } else {
            setSelectedObject(null);
            obj.transformControls.detach();
        }
    };

    const setUserData = (object, data) => {
        object.userData = [...object.userData, data];
        forceSceneUpdate();
    }

    const transformObject = (object, transformType, x, y, z) => {
        switch (transformType) {
            case "scale":
                object.scale.set(x, y, z);
                break;
            case "position":
                object.position.set(x, y, z);
                break;
            case "rotation":
                object.rotation.set(x, y, z);
                break;
            default:
                return;
        }
        forceSceneUpdate();
    }

    const copyAttributes = (object, clone) => {
        if (!object || !clone) return;

        // make the clone onBeforeRender be the same as the original
        clone.onBeforeRender = object.onBeforeRender
        clone.userData = object.userData.duplicate();
        for (let i = 0; i < object.children.length; i++) {
            copyAttributes(object.children[i], clone.children[i])
        }
    }

    const duplicateObject = (object) => {
        const clone = object.clone(true);

        //This copies the onBeforeRender callback into the clone
        copyAttributes(object, clone)

        object.parent.add(clone);
        setSelectedObject(null);
        forceSceneUpdate();
    }

    const deleteObject = (object) => {
        setSelectedObject(null);
        object.removeFromParent();
        forceSceneUpdate();
    }

    return (
        <>
            <ThreeDisplay mountRef={mountRef} />
            <ObjectParameters selectedObject={selectedObject} transformObject={transformObject} setUserData={setUserData} />
            <Toolbar setTransformMode={setTransformMode} />
            <InsertTool addObject={addObject} />
            <LinkTree scene={scene} deleteObject={deleteObject} duplicateObject={duplicateObject} selectedObject={selectedObject} selectObject={selectObject} />
            <CodeDisplay scene={scene} />
        </>
    );
}

