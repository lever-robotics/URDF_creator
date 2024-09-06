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

    // pushUndo gets called from forceCodeUpdate
    const pushUndo = async () => {
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

    // Redo stack gets destroyed when forceCodeUpdate gets called
    const popRedo = async () => {
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

    const forceSceneUpdate = () => {
        setScene({ ...threeScene.current.scene });
    };

    const setTransformMode = (selectedObject, mode) => {
        const { current: three } = threeScene;
        if (three.transformControls) {
            three.transformControls.setMode(mode);
            setToolMode(mode);
        }

        if (selectedObject) {
            selectedObject.attachTransformControls(three.transformControls);
        }
    };

    const getToolMode = () => {
        return toolMode;
    };

    const startRotateJoint = (frame) => {
        const { current: three } = threeScene;
        setToolMode("rotate");
        three.transformControls.setMode("rotate");
        frame.rotateJoint(three.transformControls);
    };

    const startMoveJoint = (frame) => {
        const { current: three } = threeScene;
        setToolMode("translate");
        three.transformControls.setMode("translate");
        frame.moveJoint(three.transformControls);
    };

    const reattachLink = (frame) => {
        const { current: three } = threeScene;
        three.transformControls.detach();
        frame.reattachLink();
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
            frame.attachTransformControls(three.transformControls);
        } else {
            setSelectedObject(null);
            three.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const setLinkColor = (frame, color) => {
        frame.color = color;
    };

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

    const setMass = (frame, mass) => {
        frame.updateMass(mass);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setInertia = (frame, type, inertia) => {
        frame.setCustomInertia(type, inertia);
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
        frame.rotateAroundJointAxis(angle);
        forceSceneUpdate();
    };

    const translateAlongJointAxis = (frame, distance) => {
        frame.translateAlongJointAxis(distance);
        forceSceneUpdate();
    };

    const saveForDisplayChanges = (frame) => {
        frame.saveForDisplayChanges();
    };

    const resetJointPosition = (frame) => {
        frame.resetJointPosition();
        forceSceneUpdate();
    };

    const setMesh = (frame, meshFileName) => {
        frame.setMesh(meshFileName);
        forceSceneUpdate();
        forceUpdateCode();
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

    const forceUpdateCode = () => {
        pushUndo();
        redo.current = [];
        setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    };

    const transformObject = (frame, transformType, axis, value) => {
        frame.operate(transformType, axis, value);
        forceSceneUpdate();
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

    const openProjectManager = () => {
        setModalContent(<ProjectDisplayer handleProjectClick={handleProjectClick} onClose={closeProjectManager} />);
        setIsModalOpen(true);
    };

    const closeProjectManager = () => {
        setIsModalOpen(false);
    };

    const closeModal = () => setIsModalOpen(false);

    // Close the onboarding Modal and launch the project manager
    const closeOnboarding = () => {
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

    const [modalContent, setModalContent] = useState(<Onboarding closeOnboarding={closeOnboarding} />);

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    const handleProjectClick = async (projectPath, title) => {
        clearScene();
        const group = await handleProject(process.env.PUBLIC_URL + projectPath);
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

    const reparentObject = (parent, child) => {
        parent.attachChild(child);
        forceUpdateCode();
    };

    const stateFunctions = {
        addObject,
        forceSceneUpdate,
        setTransformMode,
        getToolMode,
        startRotateJoint,
        startMoveJoint,
        reattachLink,
        selectObject,
        setLinkColor,
        setLinkName,

        setMass,
        setInertia,
        setSensor,
        setJointType,
        saveForDisplayChanges,
        resetJointPosition,
        translateAlongJointAxis,
        rotateAroundJointAxis,
        setJointMinMax,
        setJointValue,
        setMesh,
        updateSensor,
        loadScene,
        loadSingleObject,
        getScene,
        transformObject,
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
        setObjectPosition,
        setObjectScale,
        copyObjectScale,
        setObjectQuaternion,
        doesLinkNameExist,
        registerName,
        deregisterName,
        reparentObject,
        forceUpdateCode,
        popUndo,
        popRedo,
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
