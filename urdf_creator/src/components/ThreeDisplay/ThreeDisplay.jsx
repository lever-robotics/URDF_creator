import React, { useRef, useEffect } from "react";
import { ThreeSceneManager } from "./ThreeSceneManager";
import SceneState from "../SceneState";

function ThreeDisplay() {
    const mountRef = useRef(null);
    const threeScene = useRef(null);
    const [sceneState, stateFunctions] = SceneState({ threeScene });
    console.log(stateFunctions);

    // Set up the scene (initialization)
    useEffect(() => {
        if (!mountRef.current) return;
        const tsm = new ThreeSceneManager();
        const three = tsm.constructScene(mountRef, stateFunctions);
        threeScene.current = three;

        const sceneCallback = threeScene.current.callback;
        const setUpMouseCallback = threeScene.current.mouse.callback;

        const animate = () => {
            requestAnimationFrame(animate);
            threeScene.current.composer.render();
            threeScene.current.orbitControls.update();
        };

        animate();

        return () => {
            sceneCallback();
            setUpMouseCallback();
        };
    }, []);

    return (
        <div className="display" ref={mountRef} style={{ width: "100%", height: "100%" }}>
            {sceneState}
        </div>
    );
}

export default ThreeDisplay;
