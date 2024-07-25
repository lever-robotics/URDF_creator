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
import urdfObjectManager from "../Models/urdfObjectManager.js";

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

        const manager = new urdfObjectManager();
        const newUrdfObject = manager.createUrdfObject({
            shape: shape,
            name: shape + (numShapes[shape] + 1).toString()
        });

        // const newUrdfObject = new urdfObject(shape, shape + (numShapes[shape] + 1).toString());
        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newUrdfObject.setPosition([2.5, 2.5, 0.5]);

        if (selectedObject !== null) {
            selectedObject.attachChild(newUrdfObject);
        } else if (obj.baseLink !== null) {
            obj.baseLink.attachChild(newUrdfObject);
        } else {
            newUrdfObject.setPosition([0, 0, 0.5]);
            // newUrdfObject.setAsBaseLink(true);
            // newUrdfObject.setLinkName("base_link");
            obj.baseLink = newUrdfObject;
            obj.scene.attach(newUrdfObject);
        }
        // newUrdfObject.updateInertia();
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
        urdfObject.rotateJoint(obj.transformControls);
    };

    const startMoveJoint = (urdfObject) => {
        const { current: obj } = threeObjects;
        urdfObject.clearShimmy();
        obj.transformControls.setMode("translate");
        urdfObject.attachTransformControls(obj.transformControls);
        const worldPosition = urdfObject.linkWorldPosition();

        const lockPosition = () => {
            const currentPosition = urdfObject.linkWorldPosition();
            if (!worldPosition.equals(currentPosition)) {
                urdfObject.setGlobalPosition(worldPosition);
                forceSceneUpdate();
            }
        };

        urdfObject.addCustomRenderBehavior("lockPosition", lockPosition);
        obj.currentOffsetChangeNode = urdfObject;
    };

    const unlockCurrentOffsetChangeNode = () => {
        const { current: obj } = threeObjects;
        if (!obj.currentOffsetChangeNode) return;
        obj.currentOffsetChangeNode.clearCustomRenderBehavior('lockPosition');
        obj.currentOffsetChangeNode = null;
    };

    const selectObject = (urdfObject) => {
        const { current: obj } = threeObjects;
        unlockCurrentOffsetChangeNode();
        if (!urdfObject) {
            setSelectedObject(null);
            obj.transformControls.detach();
            return;
        }
        if (urdfObject.isSelectable()) {
            setSelectedObject(urdfObject);
            urdfObject.attachTransformControls(obj.transformControls);
        } else {
            setSelectedObject(null);
            obj.transformControls.detach();
        }
        forceSceneUpdate();
    };

    const setUserColor = (urdfObject, color) => {
        urdfObject.setColor(color);
        forceSceneUpdate();
    };

    const setMass = (urdfObject, mass) => {
        urdfObject.updateMass(mass);
        urdfObject.updateInertia();
        forceSceneUpdate();
    };

    const setInertia = (urdfObject, inertia) => {
        urdfObject.setInertia(inertia);
        forceSceneUpdate();
    };

    const setSensor = (urdfObject, sensorObj) => {
        urdfObject.setSensor(sensorObj);
        forceSceneUpdate();
    };

    const setJointType = (urdfObject, type) => {
        urdfObject.setJointType(type);
        forceSceneUpdate();
    };

    const setMesh = (urdfObject, mesh) => {
        urdfObject.setMesh(mesh);
        forceSceneUpdate();
    }

    // Loads a scene from gltf
    const loadScene = (base_link) => {
        // threeObjects.current.scene.add(scene); // This Line NEEEEEEDS to
        const baseLink = createUrdfObject(base_link);
        if (threeObjects.current.baseLink) {
            threeObjects.current.baseLink.removeFromParent()
        }
        threeObjects.current.scene.attach(baseLink);
        threeObjects.current.baseLink = baseLink;
        baseLink.setAsBaseLink = true;
        forceSceneUpdate();
        // createNewLink(threeObjects.current.scene, base_link);
    };

    const getScene = () => {
        return threeObjects.current.scene;
    };

    const transformObject = (urdfObject, transformType, x, y, z) => {
        urdfObject.operate(transformType, x, y, z);
        forceSceneUpdate();
    };

    const duplicateObject = (urdfObject) => {
        const clone = urdfObject.clone();
        urdfObject.getParent().attachChild(clone);
        setSelectedObject(null);
        forceSceneUpdate();
    };

    const deleteObject = (urdfObject) => {
        setSelectedObject(null);
        urdfObject.removeFromParent();
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
        selectObject,
        setUserColor,
        setMass,
        setInertia,
        setSensor,
        setJointType,
        loadScene,
        getScene,
        transformObject,
        duplicateObject,
        deleteObject,
        getBaseLink,
        openProjectManager,
        closeProjectManager,
        changeProjectTitle,
        unlockCurrentOffsetChangeNode,
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
                            setUserColor={setUserColor}
                            setMass={setMass}
                            setJointType={setJointType}
                            setInertia={setInertia}
                            setSensor={setSensor}
                            stateFunctions={stateFunctions}
                            setMesh={setMesh}
                        />
                        <CodeDisplay scene={scene} projectTitle={projectTitle}/>
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>
    );
}
