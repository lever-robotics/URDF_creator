import React, { useState } from "react";
import CodeBox from "./CodeBox/CodeBox";
import styles from "./RightPanel.module.css";
import ObjectParameters from "./ObjectParameters/ObjectParameters";
import ThreeScene from "../ThreeDisplay/ThreeScene";

/**
 * @param {Scene} scene
 * @param {string} projectTitle
 */

type Props = {
    projectTitle: string,
    threeScene: ThreeScene,
    updateCode: number,
    className: string
}

export default function RightPanel({ projectTitle, threeScene, updateCode }: Props) {
    const [selectedFormat, setSelectedFormat] = useState("Parameters");

    const handleClick = (e: React.MouseEvent<HTMLButtonElement>) => {
        const format = e.currentTarget.innerText;
        setSelectedFormat(format);
    };

    const selectedStyle = (format: string) => {
        if(format === selectedFormat){
            return {
                backgroundColor: "#24343f",
            }
        }
        return {}
    }

    return (
        <div className={styles.rightPanel}>
            <div className={styles.toolbar}>
                <button className={styles.toolbarButton} style={selectedStyle("Parameters")} onClick={handleClick}>
                    Parameters
                </button>
                <button className={styles.toolbarButton} style={selectedStyle("URDF")} onClick={handleClick}>
                    URDF
                </button>
                <button className={styles.toolbarButton} style={selectedStyle("SDF")} onClick={handleClick}>
                    SDF
                </button>
                <button className={styles.toolbarButton} style={selectedStyle("XACRO")} onClick={handleClick}>
                    XACRO
                </button>
            </div>
            <CodeBox projectTitle={projectTitle} threeScene={threeScene} selectedFormat={selectedFormat} updateCode={updateCode} />
            <ObjectParameters threeScene={threeScene} selectedFormat={selectedFormat} />
        </div>
    );
}
