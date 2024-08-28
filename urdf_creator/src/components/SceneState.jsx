import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import ObjectParameters from "./ObjectParameters/ObjectParameters.jsx";
import Toolbar from "./Toolbar/ToolBar.jsx";
import InsertTool from "./Insert/InsertTool.jsx";
import { LinkTree } from "./TreeView/LinkTree.jsx";
import CodeDisplay from "./CodeDisplay/CodeDisplay.jsx";
import Column from "../utils/ScreenTools/Column.jsx";
import AbsolutePosition from "../utils/ScreenTools/AbsolutePosition.jsx";
import Row from "../utils/ScreenTools/Row.jsx";
import Modal from "../FunctionalComponents/Modal.jsx";
import Onboarding from "./ApplicationHelp/Onboarding.jsx";
import ProjectDisplayer from "./ProjectManager/ProjectDisplayer.jsx";
import MenuModal from "./Menu/MenuModal.jsx";
import urdfObject from "../Models/urdfObject.jsx";
import { handleUpload, handleProject } from "../utils/HandleUpload.js";
import urdfObjectManager from "../Models/urdfObjectManager.js";

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
    };

    const setLinkColor = (urdfObject, color) => {
        urdfObject.color = color;
        forceSceneUpdate();
    };

    const setLinkName = (urdfObject, name) => {
        urdfObject.name = name;
        forceSceneUpdate();
    };

    const setMass = (urdfObject, mass) => {
        urdfObject.updateMass(mass);
        forceSceneUpdate();
    };

    const setInertia = (urdfObject, type, inertia) => {
        urdfObject.setCustomInertia(type, inertia);
        forceSceneUpdate();
    };

    const setSensor = (urdfObject, type) => {
        const manager = new urdfObjectManager();
        manager.changeSensor(urdfObject, type);
        forceSceneUpdate();
    };

    const updateSensor = (urdfObject, name, value) => {
        urdfObject.sensor.update(name, value);
        forceSceneUpdate();
    };

    const setJointType = (urdfObject, type) => {
        urdfObject.jointType = type;
        forceSceneUpdate();
    };

    const setJointMinMax = (urdfObject, type, value) => {
        urdfObject[type] = value;
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

    const resetFromDisplayChanges = (urdfObject) => {
        urdfObject.resetFromDisplayChanges();
        forceSceneUpdate();
    };

    const setMesh = (urdfObject, meshFileName) => {
        urdfObject.setMesh(meshFileName);
        forceSceneUpdate();
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

    const getScene = () => {
        const { current: three } = threeScene;
        return three.scene;
    };

    const transformObject = (urdfObject, transformType, axis, value) => {
        urdfObject.operate(transformType, axis, value);
        forceSceneUpdate();
    };

    const duplicateObject = (urdfObject) => {
        const manager = new urdfObjectManager();
        const clone = manager.cloneUrdfObject(urdfObject);

        if (urdfObject.name === "base_link") {
            clone.name = "base_link_copy";
            urdfObject.attach(clone);
        } else {
            urdfObject.parent.attach(clone);
        }
        setSelectedObject(null);
        forceSceneUpdate();
    };

    const deleteObject = (urdfObject) => {
        const { current: three } = threeScene;

        if (urdfObject.name === "base_link") {
            three.baseLink = null;
        }
        selectObject();
        urdfObject.removeFromParent();
        forceSceneUpdate();
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

    const [modalContent, setModalContent] = useState(<Onboarding closeOnboarding={closeOnboarding} />);

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    const handleProjectClick = async (projectPath, title) => {
        const group = await handleProject(process.env.PUBLIC_URL + projectPath);
        const base_link = group.scene.children[0];
        loadScene(base_link);
        setProjectTitle(title);
        setIsModalOpen(false);
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
        resetFromDisplayChanges,
        translateAlongJointAxis,
        rotateAroundJointAxis,
        setJointMinMax,
        setMesh,
        updateSensor,
        loadScene,
        getScene,
        transformObject,
        duplicateObject,
        deleteObject,
        getBaseLink,
        openProjectManager,
        openOnboarding,
        closeModal,
        changeProjectTitle,
        handleProjectClick,
    };

    return (
        <div className="screen">
            <Modal isOpen={isModalOpen} onClose={closeModal} modalContent={modalContent} />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuModal stateFunctions={stateFunctions} projectTitle={projectTitle} />
                        <LinkTree selectedObject={selectedObject} stateFunctions={stateFunctions} />
                        <InsertTool addObject={addObject} />
                    </Column>
                    <Toolbar selectedObject={selectedObject} stateFunctions={stateFunctions} />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <ObjectParameters selectedObject={selectedObject} stateFunctions={stateFunctions} />
                        <CodeDisplay scene={scene} projectTitle={projectTitle} />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>
    );
}
