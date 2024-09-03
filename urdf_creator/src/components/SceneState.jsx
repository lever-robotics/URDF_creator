import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import ObjectParameters from "./RightPanel/ObjectParameters/ObjectParameters.jsx";
import Toolbar from "./Toolbar/ToolBar.jsx";
import InsertTool from "./Insert/InsertTool.jsx";
import { LinkTree } from "./TreeView/LinkTree.jsx";
import CodeDisplay from "./RightPanel/RightPanel.jsx";
import Column from "../utils/ScreenTools/Column.jsx";
import AbsolutePosition from "../utils/ScreenTools/AbsolutePosition.jsx";
import Row from "../utils/ScreenTools/Row.jsx";
import Modal from "../FunctionalComponents/Modal.jsx";
import Onboarding from "./ApplicationHelp/Onboarding.jsx";
import ProjectDisplayer from "./ProjectManager/ProjectDisplayer.jsx";
import MenuBar from "./Menu/MenuBar.jsx";
import urdfObject from "../Models/urdfObject.jsx";
import { handleUpload, handleProject } from "../utils/HandleUpload.js";
import urdfObjectManager from "../Models/urdfObjectManager.js";
import ExportDisplayer from "./Menu/ExportModal/ExportDisplayer.jsx";
import ImportDisplayer from "./Menu/ImportModal/ImportDisplayer.jsx";
import RightPanel from "./RightPanel/RightPanel.jsx";

export default function SceneState({ threeScene }) {
    //State
    const [isModalOpen, setIsModalOpen] = useState(true);
    const [projectTitle, setProjectTitle] = useState("robot");
    const [selectedObject, setSelectedObject] = useState(null);
    const [toolMode, setToolMode] = useState("translate");
    const [scene, setScene] = useState(threeScene?.scene);
    const [numShapes, setNumShapes] = useState({
        cube: 0,
        sphere: 0,
        cylinder: 0,
    });
    const [updateCode, setUpdateCode] = useState(0);

    useEffect(() => {
        const { current: three } = threeScene;
        if (three) {
            three.mouse.addOnClickFunctions(clickObject);
        }
    }, []);

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

    const addObject = (shape) => {
        const { current: three } = threeScene;
        if (!three.scene) return;

        const manager = new urdfObjectManager();
        const newUrdfObject = manager.createUrdfObject({
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
            newUrdfObject.name = "base_link";
            three.baseLink = newUrdfObject;
            three.scene.attach(newUrdfObject);
        }
        selectObject(newUrdfObject);
        forceSceneUpdate();
        forceUpdateCode();
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
        three.transformControls.setMode("rotate");
        urdfObject.rotateJoint(three.transformControls);
    };

    const startMoveJoint = (urdfObject) => {
        const { current: three } = threeScene;
        three.transformControls.setMode("translate");
        urdfObject.moveJoint(three.transformControls);
    };

    const reattachLink = (urdfObject) => {
        const { current: three } = threeScene;
        three.transformControls.detach();
        urdfObject.reattachLink();
    };

    const selectObject = (urdfObject) => {
        console.log(urdfObject);
        const { current: three } = threeScene;
        if (!urdfObject) {
            setSelectedObject(null);
            three.transformControls.detach();
        } else if (urdfObject.selectable) {
            setSelectedObject(urdfObject);
            urdfObject.attachTransformControls(three.transformControls);
        } else {
            setSelectedObject(null);
            three.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const setLinkColor = (urdfObject, color) => {
        urdfObject.color = color;
    };

    const doesLinkNameExist = (name) => {
        const { current: three } = threeScene;
        const val = isNameDuplicate(three.baseLink, name);
        return val;
    };

    // Iterates through the tree and returns true if any object has the same provided name
    const isNameDuplicate = (urdfObject, name) => {
        if (urdfObject.name === name) {
            return true;
        } else {
            // Uses the JS .reduce() function. It's a hard to understand function but basically it iterates through every value in the array and passes an "accumulated" value to the next iteration which then modifies it and passes it on. So if a single child in the array has the same name then it will return true. Otherwise it will return false.
            return urdfObject.getUrdfObjectChildren().reduce((accumulator, currentChild) => {
                if (accumulator === true) return true;
                if (isNameDuplicate(currentChild, name)) {
                    return true;
                } else {
                    return false;
                }
            }, false);
        }
    };

    const setLinkName = (urdfObject, name) => {
        urdfObject.name = name;
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
        const manager = new urdfObjectManager();
        manager.changeSensor(urdfObject, type);
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
        urdfObject[type] = value;
        forceSceneUpdate();
        forceUpdateCode();
    };

    const setJointValue = (urdfObject, value) => {
        urdfObject.jointValue = value;
        forceSceneUpdate();
        forceUpdateCode();
    };

    const rotateAroundJointAxis = (urdfObject, angle) => {
        urdfObject.rotateAroundJointAxis(angle);
        forceSceneUpdate();
        forceUpdateCode();
    };

    const translateAlongJointAxis = (urdfObject, distance) => {
        urdfObject.translateAlongJointAxis(distance);
        forceSceneUpdate();
        forceUpdateCode();
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
        const manager = new urdfObjectManager();
        const baseLink = manager.readScene(gltfScene);
        // if (three.baseLink) {
        //     three.baseLink.removeFromParent();
        // }
        three.scene.attach(baseLink);
        three.baseLink = baseLink;
        baseLink.isBaseLink = true;
        forceSceneUpdate();
    };

    const loadSingleObject = (gltfScene) => {
        const manager = new urdfObjectManager();
        const Link = manager.readScene(gltfScene);
        if (selectedObject) {
            selectedObject.attach(Link);
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
        console.log(updateCode);
        // const update = updateCode + 1;
        setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    };

    const transformObject = (urdfObject, transformType, axis, value) => {
        urdfObject.operate(transformType, axis, value);
        forceSceneUpdate();
    };

    const duplicateObject = (urdfObject) => {
        const manager = new urdfObjectManager();
        const clone = manager.cloneUrdfObject(urdfObject);

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
        forceSceneUpdate();
        forceUpdateCode();
    };

    const getBaseLink = () => {
        const { current: three } = threeScene;
        if (three) {
            return three.baseLink;
        }
    };

    const openProjectManager = () => {
        setModalContent(<ProjectDisplayer handleProjectClick={handleProjectClick} />);
        setIsModalOpen(true);
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
        setModalContent(<ExportDisplayer onClose={closeExportDisplayer} getBaseLink={getBaseLink} projectTitle={projectTitle} getScene={getScene} />);
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
        forceSceneUpdate();
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
        isNameDuplicate,
        reparentObject,
        forceUpdateCode,
    };

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
                    <Toolbar selectedObject={selectedObject} stateFunctions={stateFunctions} />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <RightPanel scene={scene} projectTitle={projectTitle} selectedObject={selectedObject} stateFunctions={stateFunctions} updateCode={updateCode} className={"right-panel"} />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>,
        stateFunctions,
    ];
}
