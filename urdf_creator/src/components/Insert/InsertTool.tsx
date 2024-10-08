import { useState } from "react";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./Insert.module.css";
import Frame from "../../Models/Frame";

export default function InsertTool({ threeScene }: { threeScene: ThreeScene }) {
    type Mode = "Frame" | "Visual" | "Collision";
    const [selectedMode, setSelectedMode] = useState<Mode>("Frame");

    const handleModeClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        const mode = e.currentTarget.innerText;
        setSelectedMode(mode as Mode);
    };

    const handleClick = (shape: string) => {
        if(selectedMode === "Frame"){
            threeScene.addObject(shape);
            return
        }
        const selectedObject = threeScene.selectedObject instanceof Frame ? threeScene.selectedObject : threeScene.selectedObject.frame;
        if(selectedMode === "Visual"){
            selectedObject.addVisual(shape);
        }else if (selectedMode === "Collision"){
            selectedObject.addCollision(shape);
        }
        threeScene.forceUpdateBoth();
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
                    style={selectedStyle("Frame")}>
                    Frame
                </button>
                <button
                    className={styles.toolbarButton}
                    onClick={handleModeClick}
                    style={selectedStyle("Visual")}>
                    Visual
                </button>
                <button
                    className={styles.toolbarButton}
                    onClick={handleModeClick}
                    style={selectedStyle("Collision")}>
                    Collision
                </button>
            </div>
            <div className={styles.insertTool}>
                <button
                    className={styles.button}
                    onClick={() => handleClick("cube")}>
                    Add Cube
                </button>
                <button
                    className={styles.button}
                    onClick={() => handleClick("sphere")}>
                    Add Sphere
                </button>
                <button
                    className={styles.button}
                    onClick={() => handleClick("cylinder")}>
                    Add Cylinder
                </button>
            </div>
        </>
    );
}
