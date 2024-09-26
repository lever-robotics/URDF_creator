import React, { useRef, useEffect, useState } from "react";
import Toolbar from "./Toolbar/ToolBar";
import InsertTool from "./Insert/InsertTool";
import { LinkTree } from "./TreeView/LinkTree";
import Column from "../utils/ScreenTools/Column";
import AbsolutePosition from "../utils/ScreenTools/AbsolutePosition";
import Row from "../utils/ScreenTools/Row";
import Modal from "../FunctionalComponents/Modal";
import Onboarding from "./ApplicationHelp/Onboarding";
import ProjectDisplayer from "./ProjectManager/ProjectDisplayer";
import MenuBar from "./Menu/MenuBar";
import { handleProject } from "../utils/HandleUpload";
import FrameManager, { UserData } from "../Models/FrameManager";
import ExportDisplayer from "./Menu/ExportModal/ExportDisplayer";
import ImportDisplayer from "./Menu/ImportModal/ImportDisplayer";
import RightPanel from "./RightPanel/RightPanel";
import ScenetoGLTF from "../utils/ScenetoGLTF";
import { loadFileToObject } from "../utils/HandleUpload";
import * as THREE from "three";
import ThreeScene from "./ThreeDisplay/ThreeSceneObject";
import Frame, { Frameish } from "../Models/Frame";
import Mesh from "../Models/Mesh";
import { Gizmo, TransformControlsMode } from "../Models/TransformControls";

// export default function SceneState(sceneRef: React.MutableRefObject<ThreeScene | null>): [React.ReactNode, StateFunctionsType] {
    // const threeScene = sceneRef.current;
    // //State
    // const [isModalOpen, setIsModalOpen] = useState(true);
    // const [projectTitle, setProjectTitle] = useState("robot");
    // // const [selectedObject, setSelectedObject] = useState<Frameish>(undefined);
    // // const [toolMode, setToolMode] = useState("translate");
    // const [scene, setScene] = useState(threeScene?.scene);
    // // const [numShapes, setNumShapes] = useState({
    // //     cube: 0,
    // //     sphere: 0,
    // //     cylinder: 0,
    // // });
    // // const [updateCode, setUpdateCode] = useState(0);
    // type UndoState = { scene: string; selectedName: string };
    // const undo: React.MutableRefObject<UndoState[]> = useRef([
    //     {
    //         scene: "",
    //         selectedName: "",
    //     },
    // ]);

    // const redo: React.MutableRefObject<UndoState[]> = useRef([]);
    // // const objectNames: React.MutableRefObject<string[]> = useRef([]);
    // const pressedKeys = useRef<string[]>([]);

    // // very important that this function has toolMode and selected Object in its dependency array
    // // if not it will cause what is called a "stale closure"
    // // basically the function click object will save the state of toolMode and SelectedObject
    // // when it is added to the mouse's onclickfunctions
    // // this way, the function is updated whenever those values are changed 
    // // and the state stays current :)
    // useEffect(() => {
    //     const { current: three } = sceneRef;
    //     if (three) {
    //         three.mouse.addOnClickFunctions(clickObject);
    //     }
    // }, [toolMode, selectedObject]);

    // useEffect(() => {
    //     function keydown(e: KeyboardEvent): void {
    //         const pressed = pressedKeys.current;
    //         const key = e.key.toLowerCase();
    //         if (e.repeat) return; // keydown event trigger rapidly if you hold the key, we only want to detect keydown once.
    //         pressedKeys.current.push(key);

    //         // check if the keys are down
    //         const control = pressed.includes("control") || pressed.includes("meta");
    //         const shift = pressed.includes("shift");
    //         const z = pressed.includes("z");

    //         // check for undo
    //         if (control && !shift && z) {
    //             popUndo();
    //         }

    //         // check for redo
    //         if (control && shift && z) {
    //             popRedo();
    //         }
    //     }

    //     function keyup(e: KeyboardEvent) {
    //         const key = e.key.toLowerCase();
    //         const index = pressedKeys.current.indexOf(key);
    //         pressedKeys.current.splice(index, 1);
    //     }

    //     window.addEventListener("keydown", keydown);
    //     window.addEventListener("keyup", keyup);

    //     return () => {
    //         window.removeEventListener("keydown", keydown);
    //         window.removeEventListener("keyup", keyup);
    //     };
    // }, []);


    // // Function added to the Mouse object to allow clicking of meshes
    // const clickObject = (event: MouseEvent) => {
    //     const three = sceneRef.current;
    //     const rect = three?.mountRef.current?.getBoundingClientRect();
    //     const x = event.clientX - rect!.left;
    //     const y = event.clientY - rect!.top;

    //     three!.mouse.x = (x / rect!.width) * 2 - 1;
    //     three!.mouse.y = -(y / rect!.height) * 2 + 1;

    //     three!.raycaster.setFromCamera(new THREE.Vector2(three!.mouse.x, three!.mouse.y), three!.camera);
    //     const intersects = three!.raycaster.intersectObjects(three!.scene.children);

    //     // this will contain all of our objects (they have the property "isShape")
    //     const shapes: Mesh[] = intersects.filter((collision) => collision.object instanceof Mesh)
    //                                      .map((collision) => collision.object as Mesh);

    //     // this will contain all meshes in the scene (ie the transform controls)
    //     const meshes = intersects.filter((collision) => {
    //         return (collision.object.parent as Gizmo).transformType === toolMode && collision.object instanceof THREE.Mesh;
    //     });

    //     // conso.log(meshes.map((mesh) => mesh.object))

    //     // if we hit a shape, select the closest
    //     if (shapes.length > 0) {
    //         const object = shapes[0].frame;
    //         selectObject(object!);
    //         // if we don't hit any mesh (if we don't hit transform controls) deselect
    //     } else if (meshes.length === 0) {
    //         selectObject(null);
    //     }
    // }

    // /*
    //     Update Functions
    // */
    // const forceSceneUpdate = () => {
    //     // this makes a shallow copy of the scene to trick react into thinking there has been changes made
    //     // and while there have been changes they are actually nested deep into the scene tree
    //     // hence the force update

    //     // the old implementation, not working in ts
    //     // setScene({ ...sceneRef.current!.scene });

    //     // current implementation. Also not sure if it works but it will at least compile
    //     setScene(Object.create(sceneRef.current!.scene));
    // };

    // const forceUpdateCode = () => {
    //     pushUndo();
    //     redo.current = [];
    //     setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    // };

    // /*
    //     Undo/Redo Functions
    // */
    // const pushUndo = async () => {
    //     // pushUndo gets called from forceCodeUpdate
    //     const { current: undoArray } = undo;

    //     const currentScene: UndoState = {
    //         scene: "",
    //         selectedName: "",
    //     };
    //     // If the rootFrame is not null then it actually has objects so compress it
    //     if (getRootFrame() !== null) {
    //         const compressedScene = frameManager.compressScene(getRootFrame()!);
    //         const gltfScene = await ScenetoGLTF(compressedScene);
    //         currentScene.scene = JSON.stringify(gltfScene);
    //         currentScene.selectedName = selectedObject?.name!;
    //     }
    //     undoArray.push(currentScene);
    // };

    // const popUndo = async () => {
    //     const { current: three } = sceneRef;
    //     const { current: undoArray } = undo;
    //     const { current: redoArray } = redo;

    //     // There should always be an empty state as the first element
    //     if (undoArray.length === 1) return;

    //     // Pop the current state and push to the redoArray
    //     const currentState = undoArray.pop();
    //     redoArray.push(currentState!);

    //     // Get the last state before scene is cleared
    //     const lastState = undoArray[undoArray.length -1];

    //     // clear the scene
    //     clearScene();

    //     // If the last state was empty then return 
    //     if (lastState.scene === "") return;

    //     // Load the last state
    //     const lastScene = await loadFileToObject(lastState.scene, "gltf");
    //     const gltfScene = lastScene.scene;
    //     const rootFrame = frameManager.readScene(gltfScene.children[0]);

    //     three!.scene.attach(rootFrame);
    //     three!.rootFrame = rootFrame;
    //     rootFrame.isRootFrame = true;

    //     // If the lastSelected name exists then select that Object
    //     const lastSelectedName = currentState!.selectedName;
    //     const lastSelected = findFrameByName(rootFrame, lastSelectedName);
    //     selectObject(lastSelected);
    // };

    // const popRedo = async () => {
    //     // Redo stack gets destroyed when forceCodeUpdate gets called
    //     const { current: redoArray } = redo;
    //     const { current: three } = sceneRef;

    //     if (redoArray.length === 0) return;

    //     const lastState = redoArray.pop();

    //     clearScene();

    //     // If the last state was empty then return;
    //     if (lastState!.scene === "") return;

    //     const lastScene = await loadFileToObject(lastState!.scene, "gltf");
    //     const gltfScene = lastScene.scene;
    //     const rootFrame = frameManager.readScene(gltfScene.children[0]);

    //     three!.scene.attach(rootFrame);
    //     three!.rootFrame = rootFrame;
    //     rootFrame.isRootFrame = true;
    //     selectObject(null);

    //     pushUndo();
    // };

    /*
        Entire Scene Modification Functions
    */
    // const clearScene = () => {
    //     const { current: three } = sceneRef;
    //     if (three!.rootFrame === null) return;
    //     three!.rootFrame!.removeFromParent();
    //     three!.rootFrame = null;
    //     objectNames.current.length = 0;
    //     selectObject(null);
    // };

    // const addObject = (shape: string) => {
    //     const { current: three } = sceneRef;
    //     if (!three!.scene) return;

    //     const newFrame = frameManager.createFrame({
    //         shape: shape,
    //         name: shape + (numShapes[shape as keyof typeof numShapes] + 1).toString(),
    //     } as UserData);

    //     setNumShapes((prev) => ({ ...prev, [shape]: prev[shape as keyof typeof prev] + 1 }));

    //     newFrame.position.set(2.5, 2.5, 0.5);

    //     if (selectedObject) {
    //         selectedObject!.attachChild(newFrame);
    //     } else if (three!.rootFrame !== null) {
    //         three!.rootFrame!.attachChild(newFrame);
    //     } else {
    //         newFrame.position.set(0, 0, 0.5);
    //         newFrame.isRootFrame = true;
    //         setLinkName(newFrame, "base_link");
    //         three!.rootFrame = newFrame;
    //         three!.scene.attach(newFrame);
    //     }
    //     selectObject(newFrame);
    //     forceUpdateCode();
    //     forceSceneUpdate();
    // };

    // const loadScene = (gltfScene: THREE.Object3D) => {
    //     const { current: three } = sceneRef;
    //     objectNames.current.length = 0;
    //     const rootFrame = frameManager.readScene(gltfScene);
    //     // if (three.rootFrame) {
    //     //     three.rootFrame.removeFromParent();
    //     // }
    //     three!.scene.attach(rootFrame);
    //     three!.rootFrame = rootFrame;
    //     rootFrame.isRootFrame = true;
    //     closeModal();
    //     forceSceneUpdate();
    // };

    // const setTransformMode = (selectedObject: Frame, mode: string) => {
    //     const { current: three } = sceneRef;
    //     if (three!.transformControls) {
    //         three!.transformControls.setMode(mode as TransformControlsMode);
    //         setToolMode(mode);
    //     }

    //     if (selectedObject) {
    //         attachTransformControls(selectedObject);
    //     }
    // };

    // const attachTransformControls = (selectedObject: Frame) => {
    //     const { current: three } = sceneRef;
    //     const transformControls = three!.transformControls;

    //     const mode = transformControls.mode;
    //     switch (mode) {
    //         // this case will attach the transform controls to the Frame and move everything together
    //         case "translate":
    //             transformControls.attach(selectedObject);
    //             break;
    //         // will attach to Frame which will rotate the mesh about said origin
    //         case "rotate":
    //             transformControls.attach(selectedObject);
    //             break;
    //         // will attach to the link and scale nothing else
    //         case "scale":
    //             transformControls.attach(selectedObject.mesh!);
    //             break;
    //         default:
    //             break;
    //     }
    // };

    // const getToolMode = () => {
    //     return toolMode;
    // };

    // const selectObject = (frame: Frameish) => {
    //     const { current: three } = sceneRef;


    //     // the link may not be attached correctly, this checks for that case
    //     if (selectedObject?.linkDetached) {
    //         reattachLink(selectedObject!);
    //     }

    //     if (!frame) {
    //         setSelectedObject(null);
    //         three!.transformControls.detach();
    //     } else if (frame.selectable) {
    //         setSelectedObject(frame);
    //         attachTransformControls(frame);
    //     } else {
    //         setSelectedObject(null);
    //         three!.transformControls.detach();
    //     }
    //     forceSceneUpdate();
    // };

    // const loadSingleObject = (gltfScene: THREE.Object3D) => {
    //     const { current: three } = sceneRef;
    //     const frame = frameManager.readScene(gltfScene);
    //     if (selectedObject) {
    //         selectedObject.attachChild(frame);
    //     } else if (three?.rootFrame) {
    //         three.rootFrame.attachChild(frame);
    //     } else {
    //         three!.scene.attach(frame);
    //         three!.rootFrame = frame;
    //         frame.isRootFrame = true;
    //     }
    //     forceSceneUpdate();
    // };

    // const getScene = () => {
    //     const { current: three } = sceneRef;
    //     return three!.scene;
    // };

    // const duplicateObject = (frame: Frame) => {
    //     const clone = frameManager.cloneFrame(frame);

    //     if (frame.isRootFrame) {
    //         clone.parentFrame = frame;
    //         frame.attachChild(clone);
    //     } else {
    //         clone.parentFrame = frame.parentFrame;
    //         frame.parentFrame!.addChild(clone);
    //     }
    //     selectObject(clone);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const deleteObject = (frame: Frame) => {
    //     const { current: three } = sceneRef;

    //     const deleteChildren = (frame: Frame) => {
    //         frame.getFrameChildren().forEach((child: Frame) => {
    //             deleteChildren(child);
    //             child.removeFromParent();
    //             deregisterName(child.name);
    //         });
    //     };

    //     if (frame.isRootFrame) {
    //         three!.rootFrame = null;
    //     }
    //     selectObject(null);
    //     deleteChildren(frame);
    //     frame.removeFromParent();
    //     deregisterName(frame.name);
    //     forceSceneUpdate();
    // };

    // const getRootFrame = () => {
    //     const { current: three } = sceneRef;
    //     if (three) {
    //         return three!.rootFrame;
    //     }
    // };

    // const reparentObject = (parent: Frame, child: Frame) => {
    //     parent.attachChild(child);
    //     forceUpdateCode();
    // };

    /*
        Name Functions
    */
    // const doesLinkNameExist = (name: string) => {
    //     return objectNames.current.includes(name);
    // };

    // const registerName = (name: string) => {
    //     if (!doesLinkNameExist(name)) {
    //         objectNames.current.push(name);
    //         return true;
    //     }
    //     return false;
    // };

    // const deregisterName = (name: string) => {
    //     const index = objectNames.current.indexOf(name);
    //     objectNames.current.splice(index, 1);
    // };

    // const findFrameByName = (frame: Frame, name: string) => {
    //     if (frame.name === name) return frame;
    //     let returnChild = null;
    //     frame.getFrameChildren().forEach((child: Frame) => {
    //         returnChild = findFrameByName(child, name);
    //     });
    //     return returnChild;
    // };

    // const setLinkName = (frame: Frame, name: string) => {
    //     // remove old name from registry
    //     deregisterName(frame.name);

    //     //add the new name
    //     frame.name = name;
    //     frameManager.registerName(frame);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    /*
        Modal Functions
    */
    // const openProjectManager = () => {
    //     setModalContent(<ProjectDisplayer handleProjectClick={handleProjectClick} onClose={closeProjectManager} />);
    //     setIsModalOpen(true);
    // };

    // const closeProjectManager = () => {
    //     setIsModalOpen(false);
    // };

    // const closeModal = () => setIsModalOpen(false);

    // const closeOnboarding = () => {
    //     // Close the onboarding Modal and launch the project manager
    //     setIsModalOpen(false);
    //     openProjectManager();
    // };

    // const openOnboarding = () => {
    //     setModalContent(<Onboarding closeOnboarding={closeOnboarding} />);
    //     setIsModalOpen(true);
    // };

    // const closeExportDisplayer = () => {
    //     setIsModalOpen(false);
    // };

    // const openExportDisplayer = () => {
    //     setModalContent(
    //         <ExportDisplayer onClose={closeExportDisplayer} getRootFrame={getRootFrame} projectTitle={projectTitle} getScene={getScene} stateFunctions={stateFunctions} />
    //     );
    //     setIsModalOpen(true);
    // };

    // const closeImportDisplayer = () => {
    //     setIsModalOpen(false);
    // };

    // const openImportDisplayer = () => {
    //     setModalContent(<ImportDisplayer handleSensorClick={handleSensorClick} onImportClose={closeImportDisplayer} loadScene={loadScene} />);
    //     setIsModalOpen(true);
    // };

    // const changeProjectTitle = (e: React.ChangeEvent<HTMLInputElement>) => setProjectTitle(e.target!.value);

    // const handleProjectClick = async (projectPath: string, title: string) => {
    //     clearScene();
    //     const group = await handleProject(projectPath);
    //     const rootFrame = group.scene.children[0];
    //     loadScene(rootFrame);
    //     setProjectTitle(title);
    //     setIsModalOpen(false);
    // };

    // const handleSensorClick = async (gltfpath: string) => {
    //     const group = await handleProject(gltfpath);
    //     const link = group.scene.children[0];
    //     loadSingleObject(link);
    //     closeImportDisplayer();
    // };

    // const [modalContent, setModalContent] = useState(<Onboarding closeOnboarding={closeOnboarding} />);

    /*
        Frame Functions
    */
    // const startRotateJoint = (frame: Frame) => {
    //     const { current: three } = sceneRef;
    //     setToolMode("rotate");
    //     three!.transformControls.setMode("rotate");

    //     three!.transformControls.attach(frame.axis!);
    // };

    // const startMoveJoint = (frame: Frame) => {
    //     const { current: three } = sceneRef;
    //     setToolMode("translate");
    //     three!.transformControls.setMode("translate");

    //     frame.parent!.attach(frame.link!);
    //     frame.linkDetached = true;
    //     three!.transformControls.attach(frame);
    // };

    // const reattachLink = (frame: Frame) => {
    //     const { current: three } = sceneRef;
    //     three!.transformControls.detach();
    //     frame.jointVisualizer!.attach(frame.link!);
    //     frame.linkDetached = false;
    //     frame.attach(frame.axis!);
    // };

    // const setLinkColor = (frame: Frame, color: string) => {
    //     frame.setColorByHex(color);
    // };

    // const setMass = (frame: Frame, mass: number) => {
    //     frame.mass = mass;
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const setInertia = (frame: Frame, type: string, inertia: number) => {
    //     frame.inertia!.setCustomInertia(type, inertia);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const setSensor = (frame: Frame, type: string) => {
    //     frameManager.changeSensor(frame, type);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const updateSensor = (frame: Frame, name: string, value: string | number) => {
    //     frame.sensor!.update(name, value);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const setJointType = (frame: Frame, type: string) => {
    //     frame.jointType = type;
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const setJointMinMax = (frame: Frame, type: string, value: number) => {
    //     switch (type) {
    //         case "both":
    //             frame.min = -value;
    //             frame.max = value;
    //             break;
    //         case "min":
    //             frame.min = value;
    //             break;
    //         case "max":
    //             frame.max = value;
    //     }
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const setJointValue = (frame: Frame, value: number) => {
    //     frame.jointValue = value;
    //     forceSceneUpdate();
    // };

    // const rotateAroundJointAxis = (frame: Frame, angle: number) => {
    //     // Angle must be in radians
    //     // a quaternion is basically how to get from one rotation to another
    //     const quaternion = new THREE.Quaternion();

    //     // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
    //     quaternion.setFromEuler(frame.axis!.rotation);

    //     // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
    //     // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
    //     const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

    //     // the joint's rotation is then set to be a rotation around the new axis by this angle
    //     frame.jointVisualizer!.setRotationFromAxisAngle(newAxis, angle);

    //     forceSceneUpdate();
    // };

    // const translateAlongJointAxis = (frame: Frame, distance: number) => {
    //     const quaternion = new THREE.Quaternion();
    //     // a quaternion is basically how to get from one rotation to another
    //     // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
    //     quaternion.setFromEuler(frame.axis!.rotation);
    //     // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
    //     // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
    //     const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
    //     // the shimmy's rotation is then set to be a rotation around the new axis by this angle
    //     frame.jointVisualizer!.position.set(0, 0, 0);
    //     frame.jointVisualizer!.translateOnAxis(newAxis, distance);
    //     forceSceneUpdate();
    // };

    // const resetJointPosition = (frame: Frame) => {
    //     frame.jointVisualizer!.position.set(0, 0, 0);
    //     frame.jointVisualizer!.rotation.set(0, 0, 0);
    //     forceSceneUpdate();
    // };

    // const setMesh = (frame: Frame, meshFileName: string) => {
    //     frame.setMesh(meshFileName);
    //     forceSceneUpdate();
    //     forceUpdateCode();
    // };

    // const transformObject = (frame: Frame, transformType: string, axis: string, value: number) => {
    //     if (transformType === "scale") {
    //         const newValues = frame.objectScale.toArray();
    //         newValues[whichAxis(axis)] = value;
    //         frame.objectScale.set(...newValues);
    //     } else {
    //         switch (transformType) {
    //             case "position":
    //                 const newPos = frame.position.toArray();
    //                 newPos[whichAxis(axis)] = value;
    //                 frame.position.set(...newPos);
    //                 break;
    //             case "rotation":
    //                 const newRot = frame.rotation.toArray();
    //                 newRot[whichAxis(axis)] = value;
    //                 frame.rotation.set(...newRot);
    //                 break;
    //             case "scale":
    //                 const newScale = frame.rotation.toArray();
    //                 newScale[whichAxis(axis)] = value;
    //                 frame.rotation.set(...newScale);
    //                 break;
    //         }
    //     }

    //     forceUpdateCode();
    //     forceSceneUpdate();
    // };

    // const whichAxis = (axis: string) => {
    //     switch (axis) {
    //         case "x":
    //         case "radius":
    //             return 0;
    //         case "y":
    //             return 1;
    //         case "z":
    //         case "height":
    //             return 2;
    //     }
    //     return 0;
    // };

    // const setObjectPosition = (object: Frame, position: THREE.Vector3) => {
    //     object.position.copy(position);
    //     forceSceneUpdate();
    // };

    // const setObjectScale = (object: Frame, scale: THREE.Vector3) => {
    //     object.scale.set(scale.x, scale.y, scale.z);
    //     forceSceneUpdate();
    // };

    // const copyObjectScale = (object: Frame, scale: THREE.Vector3) => {
    //     object.scale.copy(scale);
    //     forceSceneUpdate();
    // };

    // const setObjectQuaternion = (object: Frame, quaternion: THREE.Quaternion) => {
    //     object.quaternion.copy(quaternion);
    //     forceSceneUpdate();
    // };


//     const stateFunctions = {
//         addObject,
//         forceSceneUpdate,
//         setTransformMode,
//         getToolMode,
//         selectObject,
//         setLinkName,
//         loadScene,
//         loadSingleObject,
//         getScene,
//         duplicateObject,
//         deleteObject,
//         getRootFrame,
//         openProjectManager,
//         openOnboarding,
//         openExportDisplayer,
//         closeExportDisplayer,
//         openImportDisplayer,
//         closeImportDisplayer,
//         closeModal,
//         changeProjectTitle,
//         handleProjectClick,
//         doesLinkNameExist,
//         registerName,
//         deregisterName,
//         reparentObject,
//         forceUpdateCode,
//         popUndo,
//         popRedo,
//         startRotateJoint,
//         startMoveJoint,
//         reattachLink,
//         setLinkColor,
//         setMass,
//         setInertia,
//         setSensor,
//         setJointType,
//         resetJointPosition,
//         translateAlongJointAxis,
//         rotateAroundJointAxis,
//         setJointMinMax,
//         setJointValue,
//         setMesh,
//         updateSensor,
//         transformObject,
//         setObjectPosition,
//         setObjectScale,
//         copyObjectScale,
//         setObjectQuaternion,
//     };

//     const frameManager = new FrameManager(stateFunctions);

//     return [
//         <div className="screen">
//             <Modal isOpen={isModalOpen} onClose={closeModal} modalContent={modalContent} />
//             <AbsolutePosition>
//                 <Row width="100%" height="100%">
//                     <Column height="100%" width="20%" pointerEvents="auto">
//                         <MenuBar stateFunctions={stateFunctions} projectTitle={projectTitle} />
//                         <LinkTree selectedObject={selectedObject} stateFunctions={stateFunctions} />
//                         <InsertTool addObject={addObject} />
//                     </Column>
//                     <Toolbar selectedObject={selectedObject} stateFunctions={stateFunctions} toolMode={toolMode} />
//                     <Column height="100%" width="25%" pointerEvents="auto">
//                         <RightPanel
//                             scene={scene!}
//                             projectTitle={projectTitle}
//                             selectedObject={selectedObject}
//                             stateFunctions={stateFunctions}
//                             updateCode={updateCode}
//                             className={"right-panel"}
//                         />
//                     </Column>
//                 </Row>
//             </AbsolutePosition>
//         </div>,
//         stateFunctions,
//     ];
// }

// to update, view this stack overflow thread: https://stackoverflow.com/questions/53113031/how-to-see-a-fully-expanded-typescript-type-without-n-more-and
// this is to fix an error within vscode that will not show the entire type definition of stateFunctions
// when you edit the file to truncate less, you will need to save as a superuser
// make sure to disable prettier! the file is massive and it will take forever to save
// then you can copy the state function type here
// export type StateFunctionsType = {
//     addObject: (shape: string) => void;
//     forceSceneUpdate: () => void;
//     setTransformMode: (selectedObject: Frame, mode: string) => void;
//     getToolMode: () => string;
//     selectObject: (frame: Frameish) => void;
//     setLinkName: (frame: Frame, name: string) => void;
//     loadScene: (gltfScene: THREE.Object3D) => void;
//     loadSingleObject: (gltfScene: THREE.Object3D) => void;
//     getScene: () => THREE.Scene;
//     duplicateObject: (frame: Frame) => void;
//     deleteObject: (frame: Frame) => void;
//     getRootFrame: () => Frameish;
//     openProjectManager: () => void;
//     openOnboarding: () => void;
//     openExportDisplayer: () => void;
//     closeExportDisplayer: () => void;
//     openImportDisplayer: () => void;
//     closeImportDisplayer: () => void;
//     closeModal: () => void;
//     changeProjectTitle: (e: React.ChangeEvent<HTMLInputElement>) => void;
//     handleProjectClick: (projectPath: string, title: string) => Promise<void>;
//     doesLinkNameExist: (name: string) => boolean;
//     registerName: (name: string) => boolean;
//     deregisterName: (name: string) => void;
//     reparentObject: (parent: Frame, child: Frame) => void;
//     forceUpdateCode: () => void;
//     popUndo: () => Promise<void>;
//     popRedo: () => Promise<void>;
//     startRotateJoint: (frame: Frame) => void;
//     startMoveJoint: (frame: Frame) => void;
//     reattachLink: (frame: Frame) => void;
//     setLinkColor: (frame: Frame, color: string) => void;
//     setMass: (frame: Frame, mass: number) => void;
//     setInertia: (frame: Frame, type: string, inertia: number) => void;
//     setSensor: (frame: Frame, type: string) => void;
//     setJointType: (frame: Frame, type: string) => void;
//     resetJointPosition: (frame: Frame) => void;
//     translateAlongJointAxis: (frame: Frame, distance: number) => void;
//     rotateAroundJointAxis: (frame: Frame, angle: number) => void;
//     setJointMinMax: (frame: Frame, type: string, value: number) => void;
//     setJointValue: (frame: Frame, value: number) => void;
//     setMesh: (frame: Frame, meshFileName: string) => void;
//     updateSensor: (frame: Frame, name: string, value: string | number) => void;
//     transformObject: (frame: Frame, transformType: string, axis: string, value: number) => void;
//     setObjectPosition: (object: Frame, position: THREE.Vector3) => void;
//     setObjectScale: (object: Frame, scale: THREE.Vector3) => void;
//     copyObjectScale: (object: Frame, scale: THREE.Vector3) => void;
//     setObjectQuaternion: (object: Frame, quaternion: THREE.Quaternion) => void;
// }
