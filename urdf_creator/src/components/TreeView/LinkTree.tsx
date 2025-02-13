import type React from "react";
import { type MutableRefObject, useEffect, useState } from "react";
import Frame, { type Frameish } from "../../Models/Frame";
import ParameterProps from "../RightPanel/ObjectParameters/ParameterProps";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import type { Selectable } from "../ThreeDisplay/ThreeScene";
import styles from "./LinkTree.module.css";
import { ObjectContextMenu } from "./ObjectContextMenu";
import TreeFrame from "./TreeFrame";

export type ContextMenu = false | Selectable;
type LinkTreeProps = {
    threeSceneRef: MutableRefObject<ThreeScene | undefined>;
};
export function LinkTree({ threeSceneRef }: LinkTreeProps) {
    const threeScene = threeSceneRef.current;

    useEffect(() => {
        if (!threeScene) return;
        const update = () => {
            setUpdate((prev) => prev + 1);
        };

        threeScene.addEventListener("addObject", update);
        threeScene.addEventListener("selectedObject", update);
        threeScene.addEventListener("name", update);

        return () => {
            threeScene.removeEventListener("addObject", update);
            threeScene.removeEventListener("selectedObject", update);
            threeScene.removeEventListener("name", update);
        };
    }, [threeScene]);

    const [contextMenuPosition, setContextMenuPosition] = useState({
        left: -1000,
        top: -10000,
    });
    const [update, setUpdate] = useState(0);

    const [hoveredFrame, setHoveredFrame] = useState<Frameish>(null);
    const [contextMenu, setContextMenu] = useState<ContextMenu>(false);

    const handleContextMenu = (
        e: React.MouseEvent,
        treeObject: ContextMenu,
    ) => {
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
    const worldFrame = threeScene?.worldFrame;
    const rootFrame = threeScene?.rootFrame;

    if (!threeScene) return;
    return (
        <>
            <div className={styles.toolbar}>
                <button className={styles.toolbarButton} type="button">
                    Link Tree
                </button>
            </div>
            <div
                className={styles.linkTree}
                onClick={onClick}
                onMouseLeave={onMouseLeave}
            >
                <div className={styles.scrollBox}>
                    <NameBar />
                    {worldFrame && (
                        <TreeFrame
                            frame={worldFrame}
                            handleContextMenu={handleContextMenu}
                            threeScene={threeScene}
                            hoveredFrame={hoveredFrame}
                            setHoveredFrame={setHoveredFrame}
                        />
                    )}
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
        </>
    );
}

function NameBar() {
    return (
        <div className={styles.nameBar}>
            {/* <div className={styles.children}>
                children
            </div>
            <div className={styles.name}>
                Name
            </div> */}
        </div>
    );
}
