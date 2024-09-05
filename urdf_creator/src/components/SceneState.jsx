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
import urdfObjectManager from "../Models/urdfObjectManager.js";
import ExportDisplayer from "./Menu/ExportModal/ExportDisplayer.jsx";
import ImportDisplayer from "./Menu/ImportModal/ImportDisplayer.jsx";
import RightPanel from "./RightPanel/RightPanel.jsx";
import ScenetoGLTF from "../utils/ScenetoGLTF.js";
import { loadFileToObject } from "../utils/HandleUpload.js";
import urdfObject from "../Models/urdfObject.jsx";

export default function SceneState({ threeScene }) {
    //State
    const [isModalOpen, setIsModalOpen] = useState(true);
    const [projectTitle, setProjectTitle] = useState("robot");
    const [selectedObject, setSelectedObject] = useState(null);
    const lastSelectedObject = useRef(null);
    console.log("up top ", lastSelectedObject.current);
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

    useEffect(() => {
        const { current: three } = threeScene;
        if (three) {
            three.mouse.addOnClickFunctions(clickObject);
        }
    }, []);

    useEffect(() => {
        const handleUndo = (e) => {
            if ((e.metaKey || e.ctrlKey) && e.key === "z" && !e.shiftKey) {
                e.preventDefault(); // Prevent the default browser undo action
                popUndo();
            } else if ((e.metaKey || e.ctrlKey) && e.key === "z" && e.shiftKey) {
                e.preventDefault();
                popRedo();
            }
        };

        window.addEventListener("keydown", handleUndo);

        return () => {
            window.removeEventListener("keydown", handleUndo);
        };
    }, [updateCode]);

    // Function added to the Mouse object to allow clicking of meshes
    function clickObject(event) {
        const three = threeScene.current;
        const rect = three.mountRef.current.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;

        three.mouse.x = (x / rect.width) * 2 - 1;
        three.mouse.y = -(y / rect.height) * 2 + 1;

        three.raycaster.setFromCamera(three.mouse, three.camera);
        const intersects = three.raycaster.intersectObjects(three.scene.children);

        const shapes = intersects.filter((collision) => collision.object.isShape);
        const meshes = intersects.filter((collision) => collision.object.type === "Mesh");

        if (shapes.length > 0) {
            const object = shapes[0].object.urdfObject;
            selectObject(object);
        } else if (meshes.length === 0) {
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
            const compressedScene = urdfManager.compressScene(getBaseLink());
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
        const baseLink = urdfManager.readScene(gltfScene.children[0]);

        three.scene.attach(baseLink);
        three.baseLink = baseLink;
        baseLink.isBaseLink = true;

        // If the lastSelected name exists then select that Object
        const lastSelectedName = currentState.selected;
        const lastSelected = findUrdfObjectByName(baseLink, lastSelectedName);
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
        const baseLink = urdfManager.readScene(gltfScene.children[0]);

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

        const newUrdfObject = urdfManager.createUrdfObject({
            shape: shape,
            name: shape + (numShapes[shape] + 1).toString(),
        });

        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newUrdfObject.position.set(2.5, 2.5, 0.5);

        if (selectedObject !== null) {
            selectedObject.link.attach(newUrdfObject);
            newUrdfObject.parentURDF = selectedObject;
        } else if (three.baseLink !== null) {
            three.baseLink.link.attach(newUrdfObject);
            newUrdfObject.parentURDF = three.baseLink;
        } else {
            newUrdfObject.position.set(0, 0, 0.5);
            newUrdfObject.isBaseLink = true;
            setLinkName(newUrdfObject, "base_link");
            three.baseLink = newUrdfObject;
            three.scene.attach(newUrdfObject);
        }
        selectObject(newUrdfObject);
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

    const startRotateJoint = (urdfObject) => {
        const { current: three } = threeScene;
        setToolMode("rotate");
        three.transformControls.setMode("rotate");
        urdfObject.rotateJoint(three.transformControls);
    };

    const startMoveJoint = (urdfObject) => {
        const { current: three } = threeScene;
        setToolMode("translate");
        three.transformControls.setMode("translate");
        urdfObject.moveJoint(three.transformControls);
    };

    const reattachLink = (urdfObject) => {
        const { current: three } = threeScene;
        three.transformControls.detach();
        urdfObject.reattachLink();
    };

    const selectObject = (urdfObject) => {
        const { current: three } = threeScene;

        console.log("last selection ", lastSelectedObject.current);
        console.log("setting to ", urdfObject);
        // the link may not be attached correctly, this checks for that case
        if (lastSelectedObject.current?.linkDetached) {
            reattachLink(lastSelectedObject.current);
            console.log("reattaching");
        }

        if (!urdfObject) {
            console.log("setting to null 1");
            setSelectedObject(null);
            three.transformControls.detach();
        } else if (urdfObject.selectable) {
            setSelectedObject(urdfObject);
            lastSelectedObject.current = urdfObject;
            urdfObject.attachTransformControls(three.transformControls);
        } else {
            console.log("setting to null 2");
            setSelectedObject(null);
            three.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const setLinkColor = (urdfObject, color) => {
        urdfObject.color = color;
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

    const findUrdfObjectByName = (urdfObject, name) => {
        if (urdfObject.name === name) return urdfObject;
        let returnChild = null;
        urdfObject.getUrdfObjectChildren().forEach((child) => {
            returnChild = findUrdfObjectByName(child, name);
        });
        return returnChild;
    };

    const setLinkName = (urdfObject, name) => {
        // remove old name from registry
        deregisterName(urdfObject.name);

        //add the new name
        urdfObject.name = name;
        urdfManager.registerName(urdfObject);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setMass = (urdfObject, mass) => {
        urdfObject.updateMass(mass);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setInertia = (urdfObject, type, inertia) => {
        urdfObject.setCustomInertia(type, inertia);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setSensor = (urdfObject, type) => {
        urdfManager.changeSensor(urdfObject, type);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const updateSensor = (urdfObject, name, value) => {
        urdfObject.sensor.update(name, value);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointType = (urdfObject, type) => {
        urdfObject.jointType = type;
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointMinMax = (urdfObject, type, value) => {
        if (type === "both") {
            urdfObject.min = -value;
            urdfObject.max = value;
        } else {
            urdfObject[type] = value;
        }
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointValue = (urdfObject, value) => {
        urdfObject.jointValue = value;
        forceSceneUpdate();
    };

    const rotateAroundJointAxis = (urdfObject, angle) => {
        urdfObject.rotateAroundJointAxis(angle);
        forceSceneUpdate();
    };

    const translateAlongJointAxis = (urdfObject, distance) => {
        urdfObject.translateAlongJointAxis(distance);
        forceSceneUpdate();
    };

    const saveForDisplayChanges = (urdfObject) => {
        urdfObject.saveForDisplayChanges();
    };

    const resetJointPosition = (urdfObject) => {
        urdfObject.resetJointPosition();
        forceSceneUpdate();
    };

    const setMesh = (urdfObject, meshFileName) => {
        urdfObject.setMesh(meshFileName);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const loadScene = (gltfScene) => {
        const { current: three } = threeScene;
        objectNames.current.length = 0;
        const baseLink = urdfManager.readScene(gltfScene);
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
        const Link = urdfManager.readScene(gltfScene);
        if (selectedObject) {
            selectedObject.link.attach(Link);
            Link.parentURDF = selectedObject;
        } else if (threeScene.current.baseLink) {
            threeScene.current.baseLink.link.attach(Link);
            Link.parentURDF = threeScene.current.baseLink;
        } else {
            const { current: three } = threeScene;
            three.scene.attach(Link);
            three.baseLink = Link;
            Link.isBaseLink = true;
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

    const transformObject = (urdfObject, transformType, axis, value) => {
        urdfObject.operate(transformType, axis, value);
        forceSceneUpdate();
    };

    const duplicateObject = (urdfObject) => {
        const clone = urdfManager.cloneUrdfObject(urdfObject);

        if (urdfObject.isBaseLink) {
            clone.parentURDF = urdfObject;
            urdfObject.link.attach(clone);
        } else {
            clone.parentURDF = urdfObject.parentURDF;
            urdfObject.parentURDF.link.add(clone);
        }
        selectObject(clone);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const deleteObject = (urdfObject) => {
        const { current: three } = threeScene;

        if (urdfObject.isBaseLink) {
            three.baseLink = null;
        }
        selectObject();
        urdfObject.removeFromParent();
        deregisterName(urdfObject.name);
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

    const setObjectQuaternion = (object, quaternion) => {
        object.quaternion.copy(quaternion);
        forceSceneUpdate();
    };

    const reparentObject = (parent, child) => {
        parent.link.attach(child);
        child.parentURDF = parent;
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
        setObjectQuaternion,
        doesLinkNameExist,
        registerName,
        deregisterName,
        reparentObject,
        forceUpdateCode,
        popUndo,
        popRedo,
    };
    const urdfManager = new urdfObjectManager(stateFunctions);

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
