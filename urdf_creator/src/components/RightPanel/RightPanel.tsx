import type React from "react";
import { type MutableRefObject, useEffect, useState } from "react";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import CodeBox from "./CodeBox/CodeBox";
import ObjectParameters from "./ObjectParameters/ObjectParameters";
import styles from "./RightPanel.module.css";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 */

type Props = {
    projectTitle: string;
    threeSceneRef: MutableRefObject<ThreeScene | undefined>;
    updateCode: number;
    className: string;
};

export default function RightPanel({
    projectTitle,
    threeSceneRef,
    updateCode,
}: Props) {
    const threeScene = threeSceneRef.current;
    const [selectedFormat, setSelectedFormat] = useState("Parameters");
    const [update, setUpdate] = useState(0);
    useEffect(() => {
        if (!threeScene) return;
        const update = () => {
            setUpdate((prev) => prev + 1);
            console.log("here");
        };

        threeScene.addEventListener("addObject", update);

        return () => {
            threeScene.removeEventListener("addObject", update);
        };
    }, [threeScene]);

    const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        const format = e.currentTarget.innerText;
        setSelectedFormat(format);
    };

    const selectedStyle = (format: string) => {
        if (format === selectedFormat) {
            return {
                // backgroundColor: "#24343f",
                backgroundColor: "#646cff",
            };
        }
        return {};
    };

    if (!threeScene) return;
    return (
        <>
            <div className={styles.toolbar}>
                <button
                    className={styles.toolbarButton}
                    style={selectedStyle("Parameters")}
                    onClick={handleClick}
                    type="button"
                >
                    Parameters
                </button>
                <button
                    className={styles.toolbarButton}
                    style={selectedStyle("URDF")}
                    onClick={handleClick}
                    type="button"
                >
                    URDF
                </button>
                <button
                    className={styles.toolbarButton}
                    style={selectedStyle("SDF")}
                    onClick={handleClick}
                    type="button"
                >
                    SDF
                </button>
                <button
                    className={styles.toolbarButton}
                    style={selectedStyle("XACRO")}
                    onClick={handleClick}
                    type="button"
                >
                    XACRO
                </button>
            </div>
            <div className={styles.rightPanel}>
                <CodeBox
                    projectTitle={projectTitle}
                    threeScene={threeScene}
                    selectedFormat={selectedFormat}
                    updateCode={updateCode}
                />
                <ObjectParameters
                    threeScene={threeScene}
                    selectedFormat={selectedFormat}
                />
            </div>
        </>
    );
}
