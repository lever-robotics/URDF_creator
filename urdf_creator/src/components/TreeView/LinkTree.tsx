import Frame, { Frameish } from "../../Models/Frame";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import { ObjectContextMenu } from "./ObjectContextMenu";
import React, { useState, useEffect } from "react";
import TreeFrame from "./TreeFrame";
import styles from "./LinkTree.module.css";

export type ContextMenu = false | Frame | Property;
export type Property = {
    name: string,
    id: number,
}
type LinkTreeProps = {
    threeScene: ThreeScene;
};
export function LinkTree({ threeScene }: LinkTreeProps) {
    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [hoveredFrame, setHoveredFrame] = useState<Frameish>(null);
    const [contextMenu, setContextMenu] = useState<ContextMenu>(false);


    const handleContextMenu = (e: React.MouseEvent, treeObject: ContextMenu) => {
        e.preventDefault();
        setContextMenu(treeObject);
        setContextMenuPosition({
            left: e.clientX,
            top: e.clientY,
        });
    };

    const onMouseLeave = () => {
        setContextMenu(false);
    };

    const onClick = () => {
        setContextMenu(false);
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
                        hoveredFrame={hoveredFrame}
                        setHoveredFrame={setHoveredFrame}
                    />
                )}
            </div>
            <ObjectContextMenu
                contextMenuPosition={contextMenuPosition}
                selectedTreeObject={contextMenu}
                threeScene={threeScene}
            />
        </div>
    );
}
