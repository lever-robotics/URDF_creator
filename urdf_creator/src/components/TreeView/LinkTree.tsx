import Frame, { Frameish } from "../../Models/Frame";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useState, useEffect } from "react";
import TreeFrame from "./TreeFrame";
import styles from "./TreeView.module.css";

// RecursiveTreeView Component
type LinkTreeProps = {
    threeScene: ThreeScene;
};
export function LinkTree({ threeScene }: LinkTreeProps) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [contextMenuVisible, setContextMenuVisible] = useState(false);
    const [hoveredButton, setHoveredButton] = useState<Frameish>(null);

    const handleContextMenu = (e: React.MouseEvent, node: Frame) => {
        e.preventDefault();
        threeScene?.selectObject(node);
        setContextMenuVisible(true);
        setContextMenuPosition({
            left: e.clientX,
            top: e.clientY,
        });
    };

    const onMouseLeave = () => {
        setContextMenuVisible(false);
    };

    const onClick = () => {
        setContextMenuVisible(false);
    };

    const rootFrame = threeScene?.rootFrame;

    return (
        <div
            className={styles.linkTree}
            onClick={onClick}
            onMouseLeave={onMouseLeave}>
            Link Tree
            <div className={styles.scrollBox}>
                {rootFrame && (
                    <TreeFrame
                        frame={rootFrame}
                        handleContextMenu={handleContextMenu}
                        threeScene={threeScene}
                        hoveredButton={hoveredButton}
                        setHoveredButton={setHoveredButton}
                    />
                )}
            </div>
            {contextMenuVisible && (
                <ObjectContextMenu
                    // objectContextMenu={objectContextMenu}
                    contextMenuPosition={contextMenuPosition}
                    selectedObject={threeScene?.selectedObject}
                    threeScene={threeScene}
                />
            )}
        </div>
    );
}
