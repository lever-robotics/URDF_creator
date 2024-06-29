import React, { useRef, useEffect, useState } from "react";
import * as THREE from "three";
import initScene from "./ThreeDisplay/InitScene.jsx";
import setUpSceneMouse from "./ThreeDisplay/SetUpMouse.jsx";
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
import Link from "../Models/Link.jsx";
import urdfObject from "../Models/urdfObject.jsx";
import { handleUpload, handleProject } from "../utils/HandleUpload.js";

export default function SceneState() {
    // State for the ProjectManager
    const [isProjectManagerOpen, setIsProjectManagerOpen] = useState(false);
    // Project Title
    const [projectTitle, setProjectTitle] = useState("robot");
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
        currentOffsetChangeNode: null,
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

        const setUpMouseCallback = setUpSceneMouse(threeObjects, mountRef, mouseData, selectObject, forceSceneUpdate);
        const sceneCallback = initScene(threeObjects, mountRef);
        setScene({ ...obj.scene });

        const animate = () => {
            requestAnimationFrame(animate);
            obj.composer.render();
            obj.orbitControls.update();
            // forceSceneUpdate();
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

        const newUrdfObject = new urdfObject(shape, shape + (numShapes[shape] + 1).toString());
        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newUrdfObject.setPosition([2.5, 2.5, 0.5]);

        if (selectedObject !== null) {
            selectedObject.attachChild(newUrdfObject);
        } else if (obj.baseLink !== null) {
            obj.baseLink.attachChild(newUrdfObject);
        } else {
            newUrdfObject.setPosition([0, 0, 0.5]);
            newUrdfObject.setAsBaseLink(true);
            newUrdfObject.setName = "base_link";
            obj.baseLink = newUrdfObject;
            obj.scene.attach(newUrdfObject);
        }
        newUrdfObject.updateInertia();
        forceSceneUpdate();
    };

    const createUrdfObject = (gltfObject) => {
        const shimmy = gltfObject.children[0] === THREE.Line ? gltfObject.children[0] : gltfObject.children[1];
        const joint = gltfObject.children[0] === THREE.Line ? gltfObject.children[1] : gltfObject.children[0];
        const link = shimmy.children[0];
        const linkChildren = link.children;
        const mesh = linkChildren.find((obj) => obj.type === "Mesh");
        const params = {
            position: gltfObject.position,
            rotation: gltfObject.rotation,
            scale: mesh.scale,
            offset: link.position,
            jointAxis: {
                type: joint.userData?.jointType ?? 'fixed',
                axis: joint.position,
                origin: [0, 0, 0], // Not sure how to do this
                name: joint.name,
            },
            jointMin: joint.userData?.min,
            jointMax: joint.userData?.max,
            jointRotation: joint.rotation,
            jointOrigin: joint.position,
            material: mesh.material,
            shape: gltfObject.userData.shape,
            userData: gltfObject.userData,
            name: gltfObject.userData.name,
        };
        const children = link.children.map((object) => {
            if (object !== THREE.Mesh) {
                return object;
            }
        });
        const object = new urdfObject(params.shape, params.name, params);
        children.forEach((child) => {
            if (child.type !== "Mesh") {
                return object.link.add(createUrdfObject(child));
            }
        });
        return object;
    };

    const forceSceneUpdate = () => {
        const { current: obj } = threeObjects;
        setScene({ ...obj.scene });
        console.log(obj.scene);
    };

    const setTransformMode = (mode, currentlySelectedObject) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }

        if (currentlySelectedObject) {
            currentlySelectedObject.attachTransformControls(obj.transformControls);
        }
    };

    const startRotateJoint = (urdfObject) => {
        const { current: obj } = threeObjects;
        obj.transformControls.setMode("rotate");
        urdfObject.startRotateJoint(obj.transformControls);
    };


    // REVIEW THE NEXT FOUR FUNCTIONS **JARED**
    const startMoveJoint = (object) => {
        const { current: obj } = threeObjects;
        object.clearShimmy();
        obj.transformControls.setMode("translate");
        obj.transformControls.attach(object);
        const worldPosition = object.link.getWorldPosition(new THREE.Vector3());

        const lockPosition = () => {
            const currentPosition = object.link.getWorldPosition(new THREE.Vector3());
            if (!worldPosition.equals(currentPosition)) {
                setGlobalPosition(object.link, worldPosition);
            }
        };

        addCustomRenderBehavior(object, "lockPosition", lockPosition);
        obj.currentOffsetChangeNode = object;
    };

    const setGlobalPosition = (object, newWorldPosition) => {
        // Get the current world matrix of the object
        const oldWorldMatrix = new THREE.Matrix4();
        oldWorldMatrix.copy(object.matrixWorld);

        // Extract the position, rotation, and scale from the world matrix
        const oldWorldPosition = new THREE.Vector3();
        const oldWorldRotation = new THREE.Quaternion();
        const oldWorldScale = new THREE.Vector3();
        oldWorldMatrix.decompose(oldWorldPosition, oldWorldRotation, oldWorldScale);

        // Compute the difference between the new world position and the old world position
        const offset = newWorldPosition.clone().sub(oldWorldPosition);

        // Transform the offset by the inverse of the object's parent's world rotation
        if (object.parent) {
            const parentWorldRotation = new THREE.Quaternion();
            object.parent.getWorldQuaternion(parentWorldRotation);
            parentWorldRotation.invert(); // Corrected method
            offset.applyQuaternion(parentWorldRotation);
        }

        // Add the transformed offset to the object's local position
        object.addOffset(offset);

        // Force the scene to update
        forceSceneUpdate();
    };

    const addCustomRenderBehavior = (object, behavior, func) => {
        object.mesh.customRenderBehaviors[behavior] = func;
    };

    const clearCustomRenderBehavior = (object, behavior) => {
        delete object.mesh.customRenderBehaviors[behavior];
    };

    const unlockCurrentOffsetChangeNode = () => {
        const { current: obj } = threeObjects;
        if (!obj.currentOffsetChangeNode) return;

        clearCustomRenderBehavior(obj.currentOffsetChangeNode, "lockPosition");
        obj.currentOffsetChangeNode = null;
    };

    const setRotationAboutJointAxis = (object, angle) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(object.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(...object.joint.axis).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        object.shimmy.setRotationFromAxisAngle(newAxis, angle);
    };

    const setPositionAcrossJointAxis = (object, distance) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(object.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        object.shimmy.position.set(0, 0, 0);
        object.shimmy.translateOnAxis(newAxis, distance);
    };

    const setJointLimits = (object, min = null, max = null) => {
        if (min !== null) {
            object.joint.min = min;
        }

        if (max !== null) {
            object.joint.max = max;
        }
        object.clearShimmy();
    };

    const selectObject = (object) => {
        const { current: obj } = threeObjects;
        unlockCurrentOffsetChangeNode();
        if (!object) {
            setSelectedObject(null);
            obj.transformControls.detach();
            return;
        }
        if (object.userData.selectable) {
            setSelectedObject(object);
            object.attachTransformControls(obj.transformControls);
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
        object.userData.inertia.updateInertia(object);
        forceSceneUpdate();
    };
    const setInertia = (object, inertia) => {
        object.userData.inertia = inertia;
        object.userData.inertia.customInertia = true;
        forceSceneUpdate();
    };

    const setSensor = (object, sensorObj) => {
        object.userData.sensor = sensorObj;
        forceSceneUpdate();
    };

    const setJoint = (object, type) => {
        object.joint.jointType = type;
        object.clearShimmy();

        if (type === "fixed") {
            object.joint.material.visible = false;
        } else {
            object.joint.material.visible = true;
        }

        if (type === "prismatic") {
            object.joint.min = -1;
            object.joint.max = 1;
        } else if (type === "revolute") {
            object.joint.min = -3.14;
            object.joint.max = 3.14;
        }

        forceSceneUpdate();
    };
    const loadScene = (base_link) => {
        // threeObjects.current.scene.add(scene); // This Line NEEEEEEDS to
        const baseLink = createUrdfObject(base_link);
        if (threeObjects.current.baseLink) {
            threeObjects.current.baseLink.removeFromParent()
        }
        threeObjects.current.scene.attach(baseLink);
        threeObjects.current.baseLink = baseLink;
        baseLink.userData.isBaseLink = true;
        forceSceneUpdate();
        // createNewLink(threeObjects.current.scene, base_link);
    };
    const getScene = () => {
        return threeObjects.current.scene;
    };

    const transformObject = (object, transformType, x, y, z) => {
        switch (transformType) {
            case "scale":
                object.mesh.scale.set(x, y, z);
                //update the moment of inertia
                object.userData.inertia.updateInertia(object);
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
    };


    const makeClone = (objectToClone) => {
        const shimmy = objectToClone.children[1]
        const joint = objectToClone.children[0]
        const link = shimmy.children[0];
        const mesh = link.children[0];
        const params = {
            position: objectToClone.position,
            rotation: objectToClone.rotation,
            scale: mesh.scale,
            offset: link.position,
            jointAxis: {
                type: joint.type,
                axis: joint.axis,
                origin: [0, 0, 0], // Not sure how to do this
                name: joint.name,
            },
            jointOrigin: joint.position,
            material: mesh.material,
            shape: objectToClone.userData.shape,
            userData: objectToClone.userData,
            name: objectToClone.userData.name + "_copy",
        };
        const children = link.children.filter((child) => child.type !== "Mesh");
        const object = new urdfObject(params.shape, params.name, params);
        children.forEach((child) => object.link.add(makeClone(child)));
        return object;
    }

    const duplicateObject = (object) => {
        const clone = makeClone(object);
        object.getParent().link.add(clone);
        setSelectedObject(null);
        forceSceneUpdate();
    };

    const deleteObject = (object) => {
        setSelectedObject(null);
        object.removeFromParent();
        forceSceneUpdate();
    };

    const getBaseLink = () => {
        return threeObjects.current.baseLink;
    };

    const openProjectManager = () => setIsProjectManagerOpen(true);
    const closeProjectManager = () => setIsProjectManagerOpen(false);

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    const handleProjectClick = async (projectPath, title) => {
        const group = await handleProject(process.env.PUBLIC_URL + projectPath);
        const base_link = group.scene.children[0];
        loadScene(base_link);
        setProjectTitle(title);
        setIsProjectManagerOpen(false);
    };

    const stateFunctions = {
        addObject,
        createUrdfObject,
        forceSceneUpdate,
        setTransformMode,
        startRotateJoint,
        startMoveJoint,
        setRotationAboutJointAxis,
        selectObject,
        setLinkName,
        setUserColor,
        setMass,
        setInertia,
        setSensor,
        setJoint,
        loadScene,
        getScene,
        transformObject,
        duplicateObject,
        deleteObject,
        getBaseLink,
        openProjectManager,
        closeProjectManager,
        changeProjectTitle,
        setPositionAcrossJointAxis,
        unlockCurrentOffsetChangeNode,
        setJointLimits,
        handleProjectClick,
    };

    return (
        <div className="screen">
            <ProjectModal isOpen={isProjectManagerOpen} onClose={closeProjectManager} handleProjectClick={handleProjectClick}/>
            <ThreeDisplay mountRef={mountRef} />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuModal
                            openProjectManager={openProjectManager}
                            changeProjectTitle={changeProjectTitle}
                            projectTitle={projectTitle}
                            getBaseLink={getBaseLink}
                            loadScene={loadScene}
                            getScene={getScene}
                        />
                        <LinkTree scene={scene} deleteObject={deleteObject} duplicateObject={duplicateObject} selectedObject={selectedObject} selectObject={selectObject} getBaseLink={getBaseLink} />
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
                            stateFunctions={stateFunctions}
                        />
                        <CodeDisplay scene={scene} />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>
    );
}
