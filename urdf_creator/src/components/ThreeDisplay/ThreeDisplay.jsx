import * as THREE from "three";
import React, { useRef, useEffect } from "react";
import InitScene from "./InitScene";
import setUpSceneMouse from "./SetUpMouse";
import SceneState from "../SceneState";

function ThreeDisplay() {
    const mountRef = useRef(null);
    const threeScene = useRef(null);

    // Set up the scene (initialization)
    useEffect(() => {
        if (!mountRef.current) return;
        threeScene.current = InitScene(mountRef);
        const three = threeScene.current;
        const sceneCallback = threeScene.current.callback;
        console.log(threeScene.current);

        const mouseData = {
            previousUpTime: null,
            currentDownTime: null,
            startPos: null,
        };

        const setUpMouseCallback = setUpSceneMouse(
            threeScene,
            mountRef,
            mouseData
        );

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
        <div
            className="display"
            ref={mountRef}
            style={{ width: "100%", height: "100%" }}>
            {/* <SceneState threeScene={threeScene.current}/> */}
        </div>
    );
}

export default ThreeDisplay;
