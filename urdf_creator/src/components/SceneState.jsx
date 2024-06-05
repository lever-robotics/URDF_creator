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
import Column from "../utils/ScreenTools/Column.jsx";
import AbsolutePosition from "../utils/ScreenTools/AbsolutePosition.jsx";
import Row from "../utils/ScreenTools/Row.jsx";
import ProjectModal from "./ProjectManager/ProjectModal.jsx";
import MenuModal from "./Menu/MenuModal.jsx";
import SceneObject from "../Models/SceneObject.jsx";

export default function SceneState() {
    // State for the ProjectManager
    const [isProjectManagerOpen, setIsProjectManagerOpen] = useState(false);
    // Project Title
    const [projectTitle, setProjectTitle] = useState("untitled");
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

    // Set up the scene (initialization)
    useEffect(() => {
        const { current: obj } = threeObjects;
        if (!mountRef.current || obj.initialized) return;

        const setUpMouseCallback = setUpSceneMouse(threeObjects, mountRef, mouseData, selectObject);
        const sceneCallback = initScene(threeObjects, mountRef);
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

        const newSceneObject = new SceneObject(shape, shape + (numShapes[shape] + 1).toString());
        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newSceneObject.position.set(2.5, 0.5, 2.5);

        if (selectedObject !== null) {
            selectedObject.attach(newSceneObject);
        } else if (obj.baseLink !== null) {
            obj.baseLink.attach(newSceneObject);
        } else {
            newSceneObject.position.set(0, 0.5, 0);
            newSceneObject.userData.isBaseLink = true;
            newSceneObject.userData.name = "base_link";
            obj.baseLink = newSceneObject;
            obj.scene.attach(newSceneObject);
        }
        console.log(obj.scene);
        forceSceneUpdate();
    };

    const createNewSceneObject = (parent, shape, name, position, rotation, scale, jointPosition, jointAxis, jointType, jointName) => {
        const object = new SceneObject(shape, name, position, rotation, scale, jointPosition, jointAxis, jointType, jointName);
        parent.add(object);
    };

    const forceSceneUpdate = () => {
        const { current: obj } = threeObjects;
        setScene({ ...obj.scene });
        console.log(scene);
    };

    const setTransformMode = (mode, currentlySelectedObject) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }
        console.log(mode);

        if (currentlySelectedObject) {
            attachTransformControls(currentlySelectedObject);
        }
    };

    const attachTransformControls = (currentlySelectedObject) => {
        const { current: obj } = threeObjects;
        const mode = obj.transformControls.mode;
        console.log(mode);
        switch (mode) {
            // this case will attach the transform controls to the joint of the object and move everything together
            case "translate":
                obj.transformControls.attach(currentlySelectedObject);
                break;
            // will attach to the origin of rotation, which will rotate the mesh about said origin
            case "rotate":
                obj.transformControls.attach(currentlySelectedObject.originOfRotation);
                break;
            // will attach to the mesh and scale nothing else
            case "scale":
                obj.transformControls.attach(currentlySelectedObject.mesh);
                break;
            default:
                break;
        }
    };

    const startRotateJoint = (object) => {
        const { current: obj } = threeObjects;
        obj.transformControls.setMode("rotate");
        obj.transformControls.attach(object.jointAxis);
    };

    const startMoveJoint = (object) => {
        const { current: obj } = threeObjects;
        obj.transformControls.setMode("translate");
        obj.transformControls.attach(object);
    };

    const selectObject = (object) => {
        const { current: obj } = threeObjects;
        if (!object) {
            setSelectedObject(null);
            obj.transformControls.detach();
            return;
        }
        if (object.userData.selectable) {
            setSelectedObject(object);
            attachTransformControls(object);
        } else {
            setSelectedObject(null);
            obj.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const setLinkName = (object, name) => {
        object.userData.name = name;
        forceSceneUpdate();
    };

    const setUserColor = (object, color) => {
        object.mesh.material.color.set(color);
        forceSceneUpdate();
    };

    const setMass = (object, mass) => {
        object.userData.inertia.updateMass(mass, object);
        forceSceneUpdate();
    };
    const setInertia = (object, inertia) => {
        object.userData.inertia = inertia;
        forceSceneUpdate();
    };

    const setSensor = (object, sensorObj) => {
        object.userData.sensor = sensorObj;
        forceSceneUpdate();
    };

    const setJoint = (object, axis) => {
        console.log("setting joint");
        object.jointAxis = axis;
        forceSceneUpdate();
    };
    const loadScene = (scene) => {
        // threeObjects.current.scene.add(scene);// This Line NEEEEEEDS to be scene.add?? Not sure why
        threeObjects.current.scene = scene;
        console.log(threeObjects.current.scene);
        forceSceneUpdate();
    };
    const getScene = () => {
        return threeObjects.current.scene;
    };

    const transformObject = (object, transformType, x, y, z) => {
        switch (transformType) {
            case "scale":
                object.mesh.scale.set(x, y, z);
                break;
            case "position":
                object.position.set(x, y, z);
                break;
            case "rotation":
                object.originOfRotation.rotation.set(x, y, z);
                break;
            default:
                return;
        }
        forceSceneUpdate();
    };

    const copyAttributes = (object, clone) => {
        if (!object || !clone) return;

        // make the clone onBeforeRender be the same as the original
        clone.onBeforeRender = object.onBeforeRender;
        clone.userData = object.userData.duplicate();
        for (let i = 0; i < object.children.length; i++) {
            copyAttributes(object.children[i], clone.children[i]);
        }
    };

    const duplicateObject = (object) => {
        const clone = object.clone(true);

        //This copies the onBeforeRender callback into the clone
        copyAttributes(object, clone);

        object.parent.add(clone);
        setSelectedObject(null);
        forceSceneUpdate();
    };

    const deleteObject = (object) => {
        setSelectedObject(null);
        object.removeFromParent();
        forceSceneUpdate();
    };

    const openProjectManager = () => setIsProjectManagerOpen(true);
    const closeProjectManager = () => setIsProjectManagerOpen(false);

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    return (
        <div className="screen">
            <ProjectModal isOpen={isProjectManagerOpen} onClose={closeProjectManager} />
            <ThreeDisplay mountRef={mountRef} />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuModal openProjectManager={openProjectManager} changeProjectTitle={changeProjectTitle} projectTitle={projectTitle} getScene={getScene} loadScene={loadScene} />
                        <LinkTree scene={scene} deleteObject={deleteObject} duplicateObject={duplicateObject} selectedObject={selectedObject} selectObject={selectObject} />
                        <InsertTool addObject={addObject} />
                    </Column>
                    <Toolbar setTransformMode={setTransformMode} selectedObject={selectedObject} />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <ObjectParameters
                            selectedObject={selectedObject}
                            transformObject={transformObject}
                            setLinkName={setLinkName}
                            setUserColor={setUserColor}
                            setMass={setMass}
                            setJoint={setJoint}
                            setInertia={setInertia}
                            setSensor={setSensor}
                            startMoveJoint={startMoveJoint}
                            startRotateJoint={startRotateJoint}
                        />
                        <CodeDisplay scene={scene} />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>
    );
}
