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

        const setUpMouseCallback = setUpSceneMouse(
            threeObjects,
            mountRef,
            mouseData,
            selectObject
        );
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

        const newUrdfObject = new urdfObject(
            shape,
            shape + (numShapes[shape] + 1).toString()
        );
        setNumShapes((prev) => ({ ...prev, [shape]: prev[shape] + 1 }));

        newUrdfObject.position.set(2.5, 0.5, 2.5);

        if (selectedObject !== null) {
            console.log(selectedObject);
            selectedObject.link.attach(newUrdfObject);
        } else if (obj.baseLink !== null) {
            obj.baseLink.link.attach(newUrdfObject);
        } else {
            newUrdfObject.position.set(0, 0.5, 0);
            newUrdfObject.userData.isBaseLink = true;
            newUrdfObject.userData.name = 'base_link';
            obj.baseLink = newUrdfObject;
            obj.scene.attach(newUrdfObject);
        }
        newUrdfObject.userData.inertia.updateInertia(newUrdfObject);
        forceSceneUpdate();
    };

    const createUrdfObject = (gltfObject) => {

        console.log(gltfObject);
        const shimmy = gltfObject.children[0] === THREE.Line ? gltfObject.children[0]: gltfObject.children[1];
        const joint = gltfObject.children[0] === THREE.Line ? gltfObject.children[1]: gltfObject.children[0];
        const link = shimmy.children[0];
        const linkChildren = link.children;
        const mesh = linkChildren.find((obj) => obj.type === 'Mesh');
        const params = {
            position: gltfObject.position,
            rotation: gltfObject.rotation,
            scale: mesh.scale,
            offset: link.position,
            jointAxis: {
                type: gltfObject.userData?.jointType ?? 'fixed',
                axis: joint.position,
                origin: [0,0,0], // Not sure how to do this
                name: joint.name,
            },
            jointOrigin: joint.position,
            material: mesh.material,
            shape: gltfObject.userData.shape,
            userData: gltfObject.userData,
            name: gltfObject.userData.name,
        };
        const children = link.children.map((object) => {
            if(object !== THREE.Mesh){
                return object;
            }
        });
        const object = new urdfObject(params.shape, params.name, params);
        children.forEach((child) => {
            if(child.type !== 'Mesh'){
                return object.link.add(createUrdfObject(child));
            }
        })
        return object;
    };

    const forceSceneUpdate = () => {
        const { current: obj } = threeObjects;
        setScene({ ...obj.scene });
    };

    const setTransformMode = (mode, currentlySelectedObject) => {
        const { current: obj } = threeObjects;
        if (obj.transformControls) {
            obj.transformControls.setMode(mode);
        }

        if (currentlySelectedObject) {
            attachTransformControls(currentlySelectedObject);
        }
    };

    const attachTransformControls = (currentlySelectedObject) => {
        const { current: obj } = threeObjects;
        const mode = obj.transformControls.mode;
        switch (mode) {
            // this case will attach the transform controls to the joint of the object and move everything together
            case 'translate':
                obj.transformControls.attach(currentlySelectedObject);
                break;
            // will attach to the origin of rotation, which will rotate the mesh about said origin
            case 'rotate':
                obj.transformControls.attach(currentlySelectedObject);
                break;
            // will attach to the mesh and scale nothing else
            case 'scale':
                obj.transformControls.attach(currentlySelectedObject.shimmy.link.mesh);
                break;
            default:
                break;
        }
    };

    const startRotateJoint = (object) => {
        const { current: obj } = threeObjects;
        obj.transformControls.setMode('rotate');
        obj.transformControls.attach(object.joint);
    };

    const startMoveJoint = (object) => {
        const { current: obj } = threeObjects;
        obj.transformControls.setMode('translate');
        obj.transformControls.attach(object);

        const worldPosition = object.link.getWorldPosition(new THREE.Vector3())

        const lockPosition = () => {
            const currentPosition = object.link.getWorldPosition(new THREE.Vector3())
            if (!worldPosition.equals(currentPosition)) {
                setGlobalPosition(object.link, worldPosition);
            }
        }

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
    }

    const clearCustomRenderBehavior = (object, behavior) => {
        delete object.mesh.customRenderBehaviors[behavior];
    }

    const unlockCurrentOffsetChangeNode = () => {
        const { current: obj } = threeObjects;
        console.log("unlocking")
        clearCustomRenderBehavior(obj.currentOffsetChangeNode, "lockPosition");
        obj.currentOffsetChangeNode = null;
    }

    const setRotationAboutJointAxis = (object, angle) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(object.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        object.shimmy.setRotationFromAxisAngle(newAxis, angle)
    }

    const setPositionAcrossJointAxis = (object, distance) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(object.joint.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        object.shimmy.position.set(0, 0, 0)
        object.shimmy.translateOnAxis(newAxis, distance)
    }

    const setJointLimits = (object, min = null, max = null) => {
        if (min) {
            object.joint.min = min;
        }

        if (max) {
            object.joint.max = max;
        }
    }

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
        object.joint.type = type;
        object.shimmy.position.set(0, 0, 0);
        object.shimmy.rotation.set(0, 0, 0);

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
            case 'scale':
                object.mesh.scale.set(x, y, z);
                //update the moment of inertia
                object.userData.inertia.updateInertia(object);
                break;
            case 'position':
                object.position.set(x, y, z);
                break;
            case 'rotation':
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

        object.parent.addByUniformScaler(clone);
        setSelectedObject(null);
        forceSceneUpdate();
    };

    const deleteObject = (object) => {
        setSelectedObject(null);
        object.removeFromParent();
        forceSceneUpdate();
    };

    const getBaseLink = () => {
        console.log(threeObjects.current.baseLink);
        return threeObjects.current.baseLink;
    }

    const openProjectManager = () => setIsProjectManagerOpen(true);
    const closeProjectManager = () => setIsProjectManagerOpen(false);

    const changeProjectTitle = (e) => setProjectTitle(e.target.value);

    const stateFunctions = {
        addObject,
        createUrdfObject,
        forceSceneUpdate,
        setTransformMode,
        attachTransformControls,
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
        copyAttributes,
        duplicateObject,
        deleteObject,
        getBaseLink,
        openProjectManager,
        closeProjectManager,
        changeProjectTitle,
        setPositionAcrossJointAxis,
        unlockCurrentOffsetChangeNode,
        setJointLimits
    };


    return (
        <div className="screen">
            <ProjectModal
                isOpen={isProjectManagerOpen}
                onClose={closeProjectManager}
            />
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
                        <LinkTree
                            scene={scene}
                            deleteObject={deleteObject}
                            duplicateObject={duplicateObject}
                            selectedObject={selectedObject}
                            selectObject={selectObject}
                            getBaseLink={getBaseLink}
                        />
                        <InsertTool addObject={addObject} />
                    </Column>
                    <Toolbar
                        setTransformMode={setTransformMode}
                        selectedObject={selectedObject}
                    />
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
