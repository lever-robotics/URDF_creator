import React, { StrictMode, useEffect, useRef, useState } from "react";
import Modal from "./FunctionalComponents/Modal";
import AbsolutePosition from "./utils/ScreenTools/AbsolutePosition";
import Row from "./utils/ScreenTools/Row";
import Column from "./utils/ScreenTools/Column";
import MenuBar from "./components/Menu/MenuBar";
import { LinkTree } from "./components/TreeView/LinkTree";
import InsertTool from "./components/Insert/InsertTool";
import Toolbar from "./components/Toolbar/ToolBar";
import RightPanel from "./components/RightPanel/RightPanel";
import ProjectDisplayer from "./components/ProjectManager/ProjectDisplayer";
import ExportDisplayer from "./components/Menu/ExportModal/ExportDisplayer";
import ImportDisplayer from "./components/Menu/ImportModal/ImportDisplayer";
import Onboarding from "./components/ApplicationHelp/Onboarding";
import initializeAnalytics from "./analytics";
import { ThreeSceneManager } from "./components/ThreeDisplay/ThreeSceneManager";
import ThreeScene from "./components/ThreeDisplay/ThreeSceneObject";
import { handleProject } from "./utils/HandleUpload";

const App = () => {
    const mountRef = useRef<HTMLDivElement>(null);
    const threeSceneRef = useRef<ThreeScene>();

    const [isModalOpen, setIsModalOpen] = useState(true);
    const [modalContent, setModalContent] = useState(<Onboarding /*closeOnboarding={closeOnboarding}*/ />);
    const [projectTitle, setProjectTitle] = useState("robot");

    const [updateCode, setUpdateCode] = useState(0);

    type UndoState = { scene: string; selectedName: string };
    const undo: React.MutableRefObject<UndoState[]> = useRef([
        {
            scene: "",
            selectedName: "",
        },
    ]);

    const redo: React.MutableRefObject<UndoState[]> = useRef([]);
    // const objectNames: React.MutableRefObject<string[]> = useRef([]);
    const pressedKeys = useRef<string[]>([]);

    // very important that this function has toolMode and selected Object in its dependency array
    // if not it will cause what is called a "stale closure"
    // basically the function click object will save the state of toolMode and SelectedObject
    // when it is added to the mouse's onclickfunctions
    // this way, the function is updated whenever those values are changed 
    // and the state stays current :)
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


    // Function added to the Mouse object to allow clicking of meshes
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

    /*
        Update Functions
    */
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


    // initialize Google Analytics
    useEffect(() => {
        initializeAnalytics();
        const handleWheel = (e: WheelEvent): void => {
            if (e.ctrlKey) {
              e.preventDefault();
            }
          };
        window.addEventListener('wheel', handleWheel, { passive: false });

        return () => {
        window.removeEventListener('wheel', handleWheel);
        };
    }, []);

    // Set up the scene (initialization)
    useEffect(() => {
        if (!mountRef.current) return;
        const tsm = new ThreeSceneManager();
        const three = tsm.constructScene(mountRef);
        threeSceneRef.current = three;

        const sceneCallback = threeSceneRef.current!.callback;
        const setUpMouseCallback = threeSceneRef.current!.mouse.callback;

        const animate = () => {
            requestAnimationFrame(animate);
            threeSceneRef.current?.composer.render();
            threeSceneRef.current?.orbitControls.update();
        };

        animate();

        return () => {
            sceneCallback();
            setUpMouseCallback();
        };
    }, []);

    const pushUndo = async () => {
        // pushUndo gets called from forceCodeUpdate
        // const { current: undoArray } = undo;

        // const currentScene: UndoState = {
        //     scene: "",
        //     selectedName: "",
        // };
        // // If the rootFrame is not null then it actually has objects so compress it
        // if (getRootFrame() !== null) {
        //     const compressedScene = frameManager.compressScene(getRootFrame()!);
        //     const gltfScene = await ScenetoGLTF(compressedScene);
        //     currentScene.scene = JSON.stringify(gltfScene);
        //     currentScene.selectedName = selectedObject?.name!;
        // }
        // undoArray.push(currentScene);
    };

    const popUndo = async () => {
        // const { current: three } = sceneRef;
        // const { current: undoArray } = undo;
        // const { current: redoArray } = redo;

        // // There should always be an empty state as the first element
        // if (undoArray.length === 1) return;

        // // Pop the current state and push to the redoArray
        // const currentState = undoArray.pop();
        // redoArray.push(currentState!);

        // // Get the last state before scene is cleared
        // const lastState = undoArray[undoArray.length -1];

        // // clear the scene
        // clearScene();

        // // If the last state was empty then return 
        // if (lastState.scene === "") return;

        // // Load the last state
        // const lastScene = await loadFileToObject(lastState.scene, "gltf");
        // const gltfScene = lastScene.scene;
        // const rootFrame = frameManager.readScene(gltfScene.children[0]);

        // three!.scene.attach(rootFrame);
        // three!.rootFrame = rootFrame;
        // rootFrame.isRootFrame = true;

        // // If the lastSelected name exists then select that Object
        // const lastSelectedName = currentState!.selectedName;
        // const lastSelected = findFrameByName(rootFrame, lastSelectedName);
        // selectObject(lastSelected);
    };

    const popRedo = async () => {
        // Redo stack gets destroyed when forceCodeUpdate gets called
        // const { current: redoArray } = redo;
        // const { current: three } = sceneRef;

        // if (redoArray.length === 0) return;

        // const lastState = redoArray.pop();

        // clearScene();

        // // If the last state was empty then return;
        // if (lastState!.scene === "") return;

        // const lastScene = await loadFileToObject(lastState!.scene, "gltf");
        // const gltfScene = lastScene.scene;
        // const rootFrame = frameManager.readScene(gltfScene.children[0]);

        // three!.scene.attach(rootFrame);
        // three!.rootFrame = rootFrame;
        // rootFrame.isRootFrame = true;
        // selectObject(null);

        // pushUndo();
    };


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
        setModalContent(<Onboarding /*closeOnboarding={closeOnboarding}*/ />);
        setIsModalOpen(true);
    };

    const closeExportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openExportDisplayer = () => {
        setModalContent(
            <ExportDisplayer onClose={closeExportDisplayer} projectTitle={projectTitle} threeScene={threeSceneRef.current!} />
        );
        setIsModalOpen(true);
    };

    const closeImportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openImportDisplayer = () => {
        setModalContent(<ImportDisplayer handleSensorClick={handleSensorClick} onImportClose={closeImportDisplayer} loadScene={threeSceneRef.current!.loadScene} />);
        setIsModalOpen(true);
    };

    const changeProjectTitle = (e: React.ChangeEvent<HTMLInputElement>) => setProjectTitle(e.target!.value);

    const handleProjectClick = async (projectPath: string, title: string) => {
        const { current: threeScene } = threeSceneRef;
        threeScene!.clearScene();
        const group = await handleProject(projectPath);
        const rootFrame = group.scene.children[0];
        threeScene!.loadScene(rootFrame);
        setProjectTitle(title);
        setIsModalOpen(false);
    };

    const handleSensorClick = async (gltfpath: string) => {
        const { current: threeScene } = threeSceneRef;
        const group = await handleProject(gltfpath);
        const link = group.scene.children[0];
        threeScene!.loadSingleObject(link);
        closeImportDisplayer();
    };

    

    const modalFunctions: ModalFunctionsType = {
        openProjectManager,
        openOnboarding,
        openExportDisplayer,
        closeExportDisplayer,
        openImportDisplayer,
        closeImportDisplayer,
        changeProjectTitle,
    }

    return (
        <div className="display" ref={mountRef} style={{ width: "100%", height: "100%" }}>
            <Modal isOpen={isModalOpen} onClose={closeModal} modalContent={modalContent} />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuBar modalFunctions={modalFunctions} projectTitle={projectTitle} />
                        <LinkTree threeScene={threeSceneRef.current!} />
                        <InsertTool threeScene={threeSceneRef.current!} />
                    </Column>
                    <Toolbar threeScene={threeSceneRef.current!} popRedo={popRedo} popUndo={popUndo} />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <RightPanel
                            projectTitle={projectTitle}
                            threeScene={threeSceneRef.current!}
                            updateCode={updateCode}
                            className={"right-panel"}
                        />
                    </Column>
                </Row>
            </AbsolutePosition>
        </div>
    );
};

export type ModalFunctionsType = {
    openProjectManager: () => void;
    openOnboarding: () => void;
    openExportDisplayer: () => void;
    closeExportDisplayer: () => void;
    openImportDisplayer: () => void;
    closeImportDisplayer: () => void;
    changeProjectTitle: (e: React.ChangeEvent<HTMLInputElement>) => void;
}

export default App;
