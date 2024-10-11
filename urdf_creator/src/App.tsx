import type React from "react";
import { StrictMode, useEffect, useRef, useState } from "react";
import Modal from "./FunctionalComponents/Modal";
import type Frame from "./Models/Frame";
import initializeAnalytics from "./analytics";
import Onboarding from "./components/ApplicationHelp/Onboarding";
import InsertTool from "./components/Insert/InsertTool";
import ExportDisplayer from "./components/Menu/ExportModal/ExportDisplayer";
import ImportDisplayer from "./components/Menu/ImportModal/ImportDisplayer";
import MenuBar from "./components/Menu/MenuBar";
import ProjectDisplayer from "./components/ProjectManager/ProjectDisplayer";
import RightPanel from "./components/RightPanel/RightPanel";
import type ThreeScene from "./components/ThreeDisplay/ThreeScene";
import { constructThreeScene } from "./components/ThreeDisplay/ThreeSceneUtils";
import {
    compressScene,
    findFrameByName,
    readScene,
} from "./components/ThreeDisplay/TreeUtils";
import Toolbar from "./components/Toolbar/ToolBar";
import { LinkTree } from "./components/TreeView/LinkTree";
import { handleProject, loadFileToObject } from "./utils/HandleUpload";
import ScenetoGLTF from "./utils/ScenetoGLTF";
import AbsolutePosition from "./utils/ScreenTools/AbsolutePosition";
import Column from "./utils/ScreenTools/Column";
import Row from "./utils/ScreenTools/Row";

const App = () => {
    const mountRef = useRef<HTMLDivElement>(null);
    const threeSceneRef = useRef<ThreeScene>();
    const [threeSceneLoaded, setThreeSceneLoaded] = useState<boolean>(false);

    const [isModalOpen, setIsModalOpen] = useState(true);
    const [modalContent, setModalContent] = useState(
        <Onboarding closeOnboarding={closeOnboarding} />,
    );
    const [projectTitle, setProjectTitle] = useState("robot");

    const [updateCode, setUpdateCode] = useState(0);
    const [updateScene, setUpdateScene] = useState(0);

    type UndoState = { scene: string; selectedName: string };
    const undo: React.MutableRefObject<UndoState[]> = useRef([
        {
            scene: "",
            selectedName: "",
        },
    ]);
    const redo: React.MutableRefObject<UndoState[]> = useRef([]);

    const pressedKeys = useRef<string[]>([]);

    // Initialize the ThreeScene ref, the animation loop, the event listeners, and google analytics
    useEffect(() => {
        if (mountRef.current === null) return;

        initializeAnalytics();

        const [sceneCallback, setUpMouseCallback] = initThreeScene(
            mountRef.current,
        );
        setThreeSceneLoaded(true);

        // Add EventListeners
        window.addEventListener("keydown", keydown);
        window.addEventListener("keyup", keyup);
        window.addEventListener("wheel", handleWheel, { passive: false });
        mountRef.current.addEventListener("updateScene", forceUpdateScene);
        mountRef.current.addEventListener("updateCode", forceUpdateCode);

        return () => {
            sceneCallback();
            setUpMouseCallback();
            window.removeEventListener("keydown", keydown);
            window.removeEventListener("keyup", keyup);
            window.removeEventListener("wheel", handleWheel);
            mountRef.current?.removeEventListener(
                "updateScene",
                forceUpdateScene,
            );
            mountRef.current?.removeEventListener(
                "updateCode",
                forceUpdateCode,
            );
        };
    }, []);

    /**
     * Handles keydown events globaly across the application
     * @param e KeyboardEvent
     */
    function keydown(e: KeyboardEvent): void {
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

    /**
     * Handles keyup events globally across the application
     * @param e KeyboardEvent
     */
    function keyup(e: KeyboardEvent) {
        const key = e.key.toLowerCase();
        const index = pressedKeys.current.indexOf(key);
        pressedKeys.current.splice(index, 1);
    }

    /**
     * Handles wheel events globally across the application
     * @param e WheelEvent
     */
    function handleWheel(e: WheelEvent): void {
        if (e.ctrlKey) {
            e.preventDefault();
        }
    }

    /**
     * Initializes the the ThreeScene
     * @param mountRef Reference to the DOM element the ThreeScene will attach to
     * @returns Callback functions for the useEffect to remove upon closure
     */
    function initThreeScene(mountDiv: HTMLDivElement) {
        const threeScene = constructThreeScene(mountDiv);
        threeSceneRef.current = threeScene;

        const sceneCallback = threeSceneRef.current.callback;
        const setUpMouseCallback = threeSceneRef.current.mouse.callback;

        const animate = () => {
            requestAnimationFrame(animate);
            threeSceneRef.current?.composer.render();
            threeSceneRef.current?.orbitControls.update();
        };

        animate();

        return [sceneCallback, setUpMouseCallback];
    }

    /*
        Update Functions
    */
    /**
     * Increments the state value updateScene by 1.
     * This causes a gobal rerender ensuring the react components are up to date with the changes made to THREE
     */
    function forceUpdateScene() {
        setUpdateScene((prev) => prev + 1);
    }

    /**
     * Increments the state value updateCode by 1.
     * This causes a gobal rerender to keep react components up to date, to update the codeBox, and to update the undo array
     */
    function forceUpdateCode() {
        pushUndo();
        redo.current = [];
        setUpdateCode((prevUpdateCode) => prevUpdateCode + 1);
    }

    /**
     * Pushes a compressed scene tree into the undo array
     * @returns Returns early if threeScene is undefined
     */
    const pushUndo = async () => {
        // pushUndo gets called from forceCodeUpdate
        const { current: threeScene } = threeSceneRef;
        const { current: undoArray } = undo;
        if (threeScene === undefined) return;
        console.log("push Undo");

        const currentScene: UndoState = {
            scene: "",
            selectedName: "",
        };
        // If the rootFrame is not null then it actually has objects so compress it
        if (threeScene.rootFrame !== null) {
            const compressedScene = compressScene(
                threeScene.rootFrame as Frame,
            );
            const gltfScene = await ScenetoGLTF(compressedScene);
            currentScene.scene = JSON.stringify(gltfScene);
            currentScene.selectedName = threeScene.selectedObject.name;
        }
        console.log(currentScene);
        undoArray.push(currentScene);
    };

    /**
     * Pops the last compressed scene tree from the undo array and loads it into THREE
     * @returns Returns early if threeScene is undefined, the undoArray is empty, or if the last scene tree was empty
     */
    const popUndo = async () => {
        const { current: threeScene } = threeSceneRef;
        const { current: undoArray } = undo;
        const { current: redoArray } = redo;
        if (threeScene === undefined) return;

        // There should always be an empty state as the first element
        if (undoArray.length === 1) return;

        // Pop the current state and push to the redoArray
        const currentState = undoArray.pop();
        if (currentState) redoArray.push(currentState);

        // Get the last state before scene is cleared
        const lastState = undoArray[undoArray.length - 1];

        // clear the scene
        threeScene.clearScene();

        // If the last state was empty then return
        console.log(undoArray);
        if (lastState.scene === "") return;

        // Load the last state
        const lastScene = await loadFileToObject(lastState.scene, "gltf");
        const gltfScene = lastScene.scene;
        const rootFrame = readScene(
            gltfScene.children[0],
            threeScene.objectNames,
        );
        console.log(rootFrame);

        threeScene.scene.attach(rootFrame);
        threeScene.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;

        // If the lastSelected name exists then select that Object
        if (currentState) {
            const lastSelectedName = currentState.selectedName;
            const lastSelected = findFrameByName(rootFrame, lastSelectedName);
            threeScene.selectObject(lastSelected ?? threeScene.worldFrame);
            return;
        }
        threeScene.selectObject(threeScene.worldFrame);
    };

    const popRedo = async () => {
        // Redo stack gets destroyed when forceCodeUpdate gets called
        const { current: redoArray } = redo;
        const { current: threeScene } = threeSceneRef;
        if (threeScene === undefined) return;

        if (redoArray.length === 0) return;

        const lastState = redoArray.pop();

        threeScene.clearScene();

        // If the last state was empty then return;
        if (!lastState) return;
        if (lastState.scene === "") return;

        const lastScene = await loadFileToObject(lastState.scene, "gltf");
        const gltfScene = lastScene.scene;
        const rootFrame = readScene(
            gltfScene.children[0],
            threeScene.objectNames,
        );

        threeScene.scene.attach(rootFrame);
        threeScene.rootFrame = rootFrame;
        rootFrame.isRootFrame = true;
        threeScene.selectObject(threeScene.worldFrame);

        pushUndo();
    };

    const openProjectManager = () => {
        setModalContent(
            <ProjectDisplayer
                handleProjectClick={handleProjectClick}
                onClose={closeProjectManager}
            />,
        );
        setIsModalOpen(true);
    };

    const closeProjectManager = () => {
        setIsModalOpen(false);
    };

    const closeModal = () => setIsModalOpen(false);

    function closeOnboarding() {
        // Close the onboarding Modal and launch the project manager
        setIsModalOpen(false);
        openProjectManager();
    }

    const openOnboarding = () => {
        setModalContent(<Onboarding closeOnboarding={closeOnboarding} />);
        setIsModalOpen(true);
    };

    const closeExportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openExportDisplayer = () => {
        if (!threeSceneRef.current) return;
        setModalContent(
            <ExportDisplayer
                onClose={closeExportDisplayer}
                projectTitle={projectTitle}
                threeScene={threeSceneRef.current}
            />,
        );
        setIsModalOpen(true);
    };

    const closeImportDisplayer = () => {
        setIsModalOpen(false);
    };

    const openImportDisplayer = () => {
        if (!threeSceneRef.current) return;
        setModalContent(
            <ImportDisplayer
                handleSensorClick={handleSensorClick}
                onImportClose={closeImportDisplayer}
                loadScene={threeSceneRef.current.loadScene}
            />,
        );
        setIsModalOpen(true);
    };

    const changeProjectTitle = (e: React.ChangeEvent<HTMLInputElement>) =>
        setProjectTitle(e.target.value);

    const handleProjectClick = async (projectPath: string, title: string) => {
        const { current: threeScene } = threeSceneRef;
        if (!threeScene) return;
        threeScene.clearScene();
        const group = await handleProject(projectPath);
        const rootFrame = group.scene.children[0];
        threeScene.loadScene(rootFrame);
        setProjectTitle(title);
        setIsModalOpen(false);
    };

    const handleSensorClick = async (gltfpath: string) => {
        const { current: threeScene } = threeSceneRef;
        if (!threeScene) return;
        const group = await handleProject(gltfpath);
        const link = group.scene.children[0];
        threeScene.loadSingleObject(link);
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
    };

    return (
        <div
            className="display"
            ref={mountRef}
            style={{ width: "100%", height: "100%" }}
        >
            <Modal
                isOpen={isModalOpen}
                onClose={closeModal}
                modalContent={modalContent}
            />
            <AbsolutePosition>
                <Row width="100%" height="100%">
                    <Column height="100%" width="20%" pointerEvents="auto">
                        <MenuBar
                            modalFunctions={modalFunctions}
                            projectTitle={projectTitle}
                        />
                        <LinkTree threeSceneRef={threeSceneRef} />
                        <InsertTool threeSceneRef={threeSceneRef} />
                    </Column>
                    <Toolbar
                        threeSceneRef={threeSceneRef}
                        popRedo={popRedo}
                        popUndo={popUndo}
                    />
                    <Column height="100%" width="25%" pointerEvents="auto">
                        <RightPanel
                            projectTitle={projectTitle}
                            threeSceneRef={threeSceneRef}
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
};

export default App;
