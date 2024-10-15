import { type MutableRefObject, useState } from "react";
import Frame, { type Shape } from "../../Models/Frame";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./Insert.module.css";

export default function InsertTool({
    threeSceneRef,
}: { threeSceneRef: MutableRefObject<ThreeScene | undefined> }) {
    if (!threeSceneRef.current) return;
    const threeScene = threeSceneRef.current;
    type Mode = "Frame" | "Visual" | "Collision";
    const [selectedMode, setSelectedMode] = useState<Mode>("Frame");

    const handleModeClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        const mode = e.currentTarget.innerText;
        setSelectedMode(mode as Mode);
    };

    const handleClick = (shape: string) => {
        if (selectedMode === "Frame") {
            threeScene.addObject(shape);
            return;
        }

        if (selectedMode === "Visual") {
            threeScene.addProperty(shape as Shape, "visual");
        } else if (selectedMode === "Collision") {
            threeScene.addProperty(shape as Shape, "collision");
        }
        threeScene.forceUpdateCode();
        return;
    };

    const selectedStyle = (mode: string) => {
        if (mode === selectedMode) {
            return {
                // backgroundColor: "#24343f",
                backgroundColor: "#646cff",
            };
        }
        return {};
    };

    return (
        <>
            <div className={styles.toolbar}>
                <button
                    className={styles.toolbarButton}
                    onClick={handleModeClick}
                    style={selectedStyle("Frame")}
                    type="button"
                >
                    Frame
                </button>
                <button
                    className={styles.toolbarButton}
                    onClick={handleModeClick}
                    style={selectedStyle("Visual")}
                    type="button"
                >
                    Visual
                </button>
                <button
                    className={styles.toolbarButton}
                    onClick={handleModeClick}
                    style={selectedStyle("Collision")}
                    type="button"
                >
                    Collision
                </button>
            </div>
            <div className={styles.insertTool}>
                <button
                    className={styles.button}
                    onClick={() => handleClick("cube")}
                    type="button"
                >
                    Add Cube
                </button>
                <button
                    className={styles.button}
                    onClick={() => handleClick("sphere")}
                    type="button"
                >
                    Add Sphere
                </button>
                <button
                    className={styles.button}
                    onClick={() => handleClick("cylinder")}
                    type="button"
                >
                    Add Cylinder
                </button>
            </div>
        </>
    );
}
