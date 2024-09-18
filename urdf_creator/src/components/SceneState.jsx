import React, { useRef, useEffect, useState } from "react";
import Toolbar from "./Toolbar/ToolBar.jsx";
import InsertTool from "./Insert/InsertTool.jsx";
import { LinkTree } from "./TreeView/LinkTree.jsx";
import Column from "../utils/ScreenTools/Column.jsx";
import AbsolutePosition from "../utils/ScreenTools/AbsolutePosition.jsx";
import Row from "../utils/ScreenTools/Row.jsx";
import Modal from "../FunctionalComponents/Modal.jsx";
import Onboarding from "./ApplicationHelp/Onboarding.jsx";
import ProjectDisplayer from "./ProjectManager/ProjectDisplayer.jsx";
import MenuBar from "./Menu/MenuBar.jsx";
import { handleProject } from "../utils/HandleUpload.js";
import FrameManager from "../Models/FrameManager.js";
import ExportDisplayer from "./Menu/ExportModal/ExportDisplayer.jsx";
import ImportDisplayer from "./Menu/ImportModal/ImportDisplayer.jsx";
import RightPanel from "./RightPanel/RightPanel.jsx";
import ScenetoGLTF from "../utils/ScenetoGLTF.js";
import { loadFileToObject } from "../utils/HandleUpload.js";
import * as THREE from "three";

export default function SceneState({ threeScene }) {
    //State
    const [isModalOpen, setIsModalOpen] = useState(true);
    const [projectTitle, setProjectTitle] = useState("robot");
    const [selectedObject, setSelectedObject] = useState(null);
    const lastSelectedObject = useRef(null);
    const [toolMode, setToolMode] = useState("translate");
    const [scene, setScene] = useState(threeScene?.scene);
    const [numShapes, setNumShapes] = useState({
        cube: 0,
        sphere: 0,
        cylinder: 0,
    });
    const [updateCode, setUpdateCode] = useState(0);
    const undo = useRef([
        {
            scene: "",
            selected: null,
        },
    ]);
    const redo = useRef([]);
    const objectNames = useRef([]);
    const pressedKeys = useRef([]);
    useEffect(() => {
        const { current: three } = threeScene;
        if (three) {
            three.mouse.addOnClickFunctions(clickObject);
        }
    }, []);

    useEffect(() => {
        function keydown(e) {
            const pressed = pressedKeys.current;
            const key = e.key.toLowerCase();
            if (e.repeat) return; // keydown event trigger rapidly if you hold the key, we only want to detect keydown once.
            pressedKeys.current.push(key);

            // check if the keys are down
            const control = pressed.includes("control") || pressed.includes("meta");
            const shift = pressed.includes("shift");
            const z = pressed.includes("z");

            // check for undo
            if (control && !shift && z) {
                popUndo();
            }

            // check for redo
            if (control && shift && z) {
                popRedo();
            }
        }

        function keyup(e) {
            const key = e.key.toLowerCase();
            const index = pressedKeys.current.indexOf(key);
            pressedKeys.current.splice(index, 1);
        }

        window.addEventListener("keydown", keydown);
        window.addEventListener("keyup", keyup);

        return () => {
            window.removeEventListener("keydown", keydown);
            window.removeEventListener("keyup", keyup);
        };
    }, []);

    useEffect(() => {}, []);

    // Function added to the Mouse object to allow clicking of meshes
    function clickObject(event) {
        console.log(event);
        const three = threeScene.current;
        const rect = three.mountRef.current.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;

        three.mouse.x = (x / rect.width) * 2 - 1;
        three.mouse.y = -(y / rect.height) * 2 + 1;

        three.raycaster.setFromCamera(three.mouse, three.camera);
        const intersects = three.raycaster.intersectObjects(three.scene.children);

        // this will contain all of our objects (they have the property "isShape")
        const shapes = intersects.filter((collision) => collision.object.isShape);

        // this will contain all meshes in the scene (ie the transform controls)
        const meshes = intersects.filter((collision) => {
            return collision.object.parent.transformType === toolMode && collision.object.isMesh;
        });

        console.log(meshes.map((val) => val.object));

        // if we hit a shape, select the closest
        if (shapes.length > 0) {
            const object = shapes[0].object.frame;
            console.log("selecting shape");
            selectObject(object);
            // if we don't hit any mesh (if we don't hit transform controls) deselect
        } else if (meshes.length === 0) {
            console.log("nothing selected");
            selectObject(null);
        }
    }


    /*
        Update Functions
    */    
    const forceSceneUpdate = () => {
        setScene({ ...threeScene.current.scene });
    };
    
    const forceUpdateCode = () => {
        pushUndo();
        redo.current = [];
        setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    };


    /*
        Undo/Redo Functions
    */
    const pushUndo = async () => {
        // pushUndo gets called from forceCodeUpdate
        const { current: undoArray } = undo;

        const currentScene = {
            scene: "",
            selected: null,
        };
        // If the baseLink is not null then it actually has objects so compress it
        if (getBaseLink() !== null) {
            const compressedScene = frameManager.compressScene(getBaseLink());
            const gltfScene = await ScenetoGLTF(compressedScene);
            currentScene.scene = JSON.stringify(gltfScene);
            currentScene.selected = selectedObject?.name;
        }
        undoArray.push(currentScene);
    };

    const popUndo = async () => {
        const { current: three } = threeScene;
        const { current: undoArray } = undo;
        const { current: redoArray } = redo;

        // There should always be an empty state as the first element
        if (undoArray.length === 1) return;

        // Pop the current state and push to the redoArray
        const currentState = undoArray.pop();
        redoArray.push(currentState);

        // Get the last state but leave it in the redoArray
        const lastState = undoArray[undoArray.length - 1];

        // If the last state was empty then clear the scene
        if (lastState.scene === "") {
            clearScene();
            return;
        }

        clearScene();
        // Load the last state
        const lastScene = await loadFileToObject(lastState.scene, "gltf");
        const gltfScene = lastScene.scene;
        const baseLink = frameManager.readScene(gltfScene.children[0]);

        three.scene.attach(baseLink);
        three.baseLink = baseLink;
        baseLink.isBaseLink = true;

        // If the lastSelected name exists then select that Object
        const lastSelectedName = currentState.selected;
        const lastSelected = findFrameByName(baseLink, lastSelectedName);
        selectObject(lastSelected);
    };

    const popRedo = async () => {
        // Redo stack gets destroyed when forceCodeUpdate gets called
        const { current: redoArray } = redo;
        const { current: three } = threeScene;

        if (redoArray.length === 0) return;

        const lastState = redoArray.pop();

        // If the last state was empty then clear the scene
        if (lastState.scene === "") {
            clearScene();
            return;
        }

        clearScene();

        const lastScene = await loadFileToObject(lastState.scene, "gltf");
        const gltfScene = lastScene.scene;
        const baseLink = frameManager.readScene(gltfScene.children[0]);

        three.scene.attach(baseLink);
        three.baseLink = baseLink;
        baseLink.isBaseLink = true;
        selectObject(null);

        pushUndo();
    };


    /*
        Entire Scene Modification Functions
    */
    const clearScene = () => {
        const { current: three } = threeScene;
        if (three.baseLink === null) return;
        three.baseLink.removeFromParent();
        three.baseLink = null;
        objectNames.current.length = 0;
        selectObject(null);
    };

    const addObject = (shape) => {
        const { current: three } = threeScene;
        if (!three.scene) return;

        const newFrame = frameManager.createFrame({
            shape: shape,
            name: shape + (numShapes[shape] + 1).toString(),
        });

        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newFrame.position.set(2.5, 2.5, 0.5);

        if (selectedObject !== null) {
            selectedObject.attachChild(newFrame);
        } else if (three.baseLink !== null) {
            three.baseLink.attachChild(newFrame);
        } else {
            newFrame.position.set(0, 0, 0.5);
            newFrame.isBaseLink = true;
            setLinkName(newFrame, "base_link");
            three.baseLink = newFrame;
            three.scene.attach(newFrame);
        }
        selectObject(newFrame);
        forceUpdateCode();
        forceSceneUpdate();
    };

    const loadScene = (gltfScene) => {
        const { current: three } = threeScene;
        objectNames.current.length = 0;
        const baseLink = frameManager.readScene(gltfScene);
        // if (three.baseLink) {
        //     three.baseLink.removeFromParent();
        // }
        three.scene.attach(baseLink);
        three.baseLink = baseLink;
        baseLink.isBaseLink = true;
        closeModal();
        forceSceneUpdate();
    };

    const setTransformMode = (selectedObject, mode) => {
        const { current: three } = threeScene;
        if (three.transformControls) {
            three.transformControls.setMode(mode);
            setToolMode(mode);
        }

        if (selectedObject) {
            attachTransformControls(selectedObject);
        }
    };

    const attachTransformControls = (selectedObject) => {
        const { current: three } = threeScene;
        const transformControls = three.transformControls;

        const mode = transformControls.mode;
        switch (mode) {
            // this case will attach the transform controls to the Frame and move everything together
            case "translate":
                transformControls.attach(selectedObject);
                break;
            // will attach to Frame which will rotate the mesh about said origin
            case "rotate":
                transformControls.attach(selectedObject);
                break;
            // will attach to the link and scale nothing else
            case "scale":
                transformControls.attach(selectedObject.mesh);
                break;
            default:
                break;
        }
    };

    const getToolMode = () => {
        return toolMode;
    };

    const selectObject = (frame) => {
        const { current: three } = threeScene;

        // the link may not be attached correctly, this checks for that case
        if (lastSelectedObject.current?.linkDetached) {
            reattachLink(lastSelectedObject.current);
        }

        if (!frame) {
            setSelectedObject(null);
            three.transformControls.detach();
        } else if (frame.selectable) {
            setSelectedObject(frame);
            lastSelectedObject.current = frame;
            attachTransformControls(frame);
        } else {
            setSelectedObject(null);
            three.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const loadSingleObject = (gltfScene) => {
        const frame = frameManager.readScene(gltfScene);
        if (selectedObject) {
            selectedObject.attachChild(frame);
        } else if (threeScene.current.baseLink) {
            threeScene.current.baseLink.attachChild(frame);
        } else {
            const { current: three } = threeScene;
            three.scene.attach(frame);
            three.baseLink = frame;
            frame.isBaseLink = true;
        }
        forceSceneUpdate();
    };

    const getScene = () => {
        const { current: three } = threeScene;
        return three.scene;
    };

    const duplicateObject = (frame) => {
        const clone = frameManager.cloneFrame(frame);

        if (frame.isBaseLink) {
            clone.parentFrame = frame;
            frame.attachChild(clone);
        } else {
            clone.parentFrame = frame.parentFrame;
            frame.parentFrame.addChild(clone);
        }
        selectObject(clone);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const deleteObject = (frame) => {
        const { current: three } = threeScene;

        const deleteChildren = (frame) => {
            frame.getFrameChildren().forEach((child) => {
                deleteChildren(child);
                child.removeFromParent();
                deregisterName(child.name);
            });
        };

        if (frame.isBaseLink) {
            three.baseLink = null;
        }
        selectObject();
        deleteChildren(frame);
        frame.removeFromParent();
        deregisterName(frame.name);
        forceSceneUpdate();
    };

    const getBaseLink = () => {
        const { current: three } = threeScene;
        if (three) {
            return three.baseLink;
        }
    };

    const reparentObject = (parent, child) => {
        parent.attachChild(child);
        forceUpdateCode();
    };


    /*
        Name Functions
    */
    const doesLinkNameExist = (name) => {
        return objectNames.current.includes(name);
    };

    const registerName = (name) => {
        if (!doesLinkNameExist(name)) {
            objectNames.current.push(name);
            return true;
        }
        return false;
    };

    const deregisterName = (name) => {
        const index = objectNames.current.indexOf(name);
        objectNames.current.splice(index, 1);
    };

    const findFrameByName = (frame, name) => {
        if (frame.name === name) return frame;
        let returnChild = null;
        frame.getFrameChildren().forEach((child) => {
            returnChild = findFrameByName(child, name);
        });
        return returnChild;
    };

    const setLinkName = (frame, name) => {
        // remove old name from registry
        deregisterName(frame.name);

        //add the new name
        frame.name = name;
        frameManager.registerName(frame);
        forceSceneUpdate();
        forceUpdateCode();
    };


    /*
        Modal Functions
    */
    const openProjectManager = () => {
        setModalContent(<ProjectDisplayer handleProjectClick={handleProjectClick} onClose={closeProjectManager} />);
        setIsModalOpen(true);
    };

    const closeProjectManager = () => {
        setIsModalOpen(false);
    };

    const closeModal = () => setIsModalOpen(false);

    const closeOnboarding = () => {
        // Close the onboarding Modal and launch the project manager
        setIsModalOpen(false);
        openProjectManager();
    };

    const openOnboarding = () => {
        setModalContent(<Onboarding closeOnboarding={closeOnboarding} />);
        setIsModalOpen(true);
    };

    const closeExportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openExportDisplayer = () => {
        setModalContent(<ExportDisplayer onClose={closeExportDisplayer} getBaseLink={getBaseLink} projectTitle={projectTitle} getScene={getScene} stateFunctions={stateFunctions} />);
        setIsModalOpen(true);
    };

    const closeImportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openImportDisplayer = () => {
        setModalContent(<ImportDisplayer handleSensorClick={handleSensorClick} onImportClose={closeImportDisplayer} loadScene={loadScene} />);
        setIsModalOpen(true);
    };

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    const handleProjectClick = async (projectPath, title) => {
        clearScene();
        const group = await handleProject(projectPath);
        const baseLink = group.scene.children[0];
        loadScene(baseLink);
        setProjectTitle(title);
        setIsModalOpen(false);
    };

    const handleSensorClick = async (gltfpath) => {
        const group = await handleProject(gltfpath);
        const link = group.scene.children[0];
        loadSingleObject(link);
        closeImportDisplayer();
    };

    const [modalContent, setModalContent] = useState(<Onboarding closeOnboarding={closeOnboarding} />);


    /*
        Frame Functions
    */
    const startRotateJoint = (frame) => {
        const { current: three } = threeScene;
        setToolMode("rotate");
        three.transformControls.setMode("rotate");

        three.transformControls.attach(frame.axis);
    };

    const startMoveJoint = (frame) => {
        const { current: three } = threeScene;
        setToolMode("translate");
        three.transformControls.setMode("translate");

        frame.parent.attach(frame.link);
        frame.linkDetached = true;
        three.transformControls.attach(frame);
    };

    const reattachLink = (frame) => {
        const { current: three } = threeScene;
        three.transformControls.detach();
        frame.jointVisualizer.attach(frame.link);
        frame.linkDetached = false;
        frame.attach(frame.axis);
    };

    const setLinkColor = (frame, color) => {
        frame.color = color;
    };

    const setMass = (frame, mass) => {
        frame.mass = mass;
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setInertia = (frame, type, inertia) => {
        frame.inertia.setCustomInertia(type, inertia);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setSensor = (frame, type) => {
        frameManager.changeSensor(frame, type);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const updateSensor = (frame, name, value) => {
        frame.sensor.update(name, value);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointType = (frame, type) => {
        frame.jointType = type;
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointMinMax = (frame, type, value) => {
        if (type === "both") {
            frame.min = -value;
            frame.max = value;
        } else {
            frame[type] = value;
        }
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointValue = (frame, value) => {
        frame.jointValue = value;
        forceSceneUpdate();
    };

    const rotateAroundJointAxis = (frame, angle) => {
        // Angle must be in radians
        // a quaternion is basically how to get from one rotation to another
        const quaternion = new THREE.Quaternion();

        // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
        quaternion.setFromEuler(frame.axis.rotation);

        // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

        // the joint's rotation is then set to be a rotation around the new axis by this angle
        frame.jointVisualizer.setRotationFromAxisAngle(newAxis, angle);

        forceSceneUpdate();
    };

    const translateAlongJointAxis = (frame, distance) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(frame.axis.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        frame.jointVisualizer.position.set(0, 0, 0);
        frame.jointVisualizer.translateOnAxis(newAxis, distance);
        forceSceneUpdate();
    };

    const resetJointPosition = (frame) => {
        frame.jointVisualizer.position.set(0, 0, 0);
        frame.jointVisualizer.rotation.set(0, 0, 0);
        forceSceneUpdate();
    };

    const setMesh = (frame, meshFileName) => {
        frame.setMesh(meshFileName);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const transformObject = (frame, transformType, axis, value) => {
        if (transformType === "scale") {
            const newValues = frame.objectScale.toArray();
            newValues[whichAxis(axis)] = value;
            frame.objectScale.set(...newValues);
        } else {
            const newValues = frame[transformType].toArray();
            newValues[whichAxis(axis)] = value;
            frame[transformType].set(...newValues);
        }

        forceUpdateCode();
        forceSceneUpdate();
    };

    const whichAxis = (axis) => {
        try {
            switch (axis) {
                case "x":
                case "radius":
                    return 0;
                case "y":
                    return 1;
                case "z":
                case "height":
                    return 2;
                default:
                    throw new Error(
                        "Axis must be 'x', 'y', 'z', 'radius, or 'height'"
                    );
            }
        } catch (e) {
            console.error(e, "axis provided", axis);
        }
    }

    const setObjectPosition = (object, position) => {
        object.position.copy(position);
        forceSceneUpdate();
    };

    const setObjectScale = (object, scale) => {
        object.scale.set(scale.x, scale.y, scale.z);
        forceSceneUpdate();
    };

    const copyObjectScale = (object, scale) => {
        object.scale.copy(scale);
        forceSceneUpdate();
    };

    const setObjectQuaternion = (object, quaternion) => {
        object.quaternion.copy(quaternion);
        forceSceneUpdate();
    };



    const stateFunctions = {
        addObject,
        forceSceneUpdate,
        setTransformMode,
        getToolMode,
        selectObject,
        setLinkName,
        loadScene,
        loadSingleObject,
        getScene,
        duplicateObject,
        deleteObject,
        getBaseLink,
        openProjectManager,
        openOnboarding,
        openExportDisplayer,
        closeExportDisplayer,
        openImportDisplayer,
        closeImportDisplayer,
        closeModal,
        changeProjectTitle,
        handleProjectClick,
        doesLinkNameExist,
        registerName,
        deregisterName,
        reparentObject,
        forceUpdateCode,
        popUndo,
        popRedo,
        startRotateJoint,
        startMoveJoint,
        reattachLink,
        setLinkColor,    
        setMass,
        setInertia,
        setSensor,
        setJointType,
        resetJointPosition,
        translateAlongJointAxis,
        rotateAroundJointAxis,
        setJointMinMax,
        setJointValue,
        setMesh,
        updateSensor,
        transformObject,
        setObjectPosition,
        setObjectScale,
        copyObjectScale,
        setObjectQuaternion,
    };
    const frameManager = new FrameManager(stateFunctions);


    return [
        <div className="screen">
            <Modal isOpen={isModalOpen} onClose={closeModal} modalContent={modalContent} />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuBar stateFunctions={stateFunctions} projectTitle={projectTitle} />
                        <LinkTree selectedObject={selectedObject} stateFunctions={stateFunctions} />
                        <InsertTool addObject={addObject} />
                    </Column>
                    <Toolbar selectedObject={selectedObject} stateFunctions={stateFunctions} toolMode={toolMode} />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <RightPanel scene={scene} projectTitle={projectTitle} selectedObject={selectedObject} stateFunctions={stateFunctions} updateCode={updateCode} className={"right-panel"} />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>,
        stateFunctions,
    ];
}
