import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./TreeView.module.css";

type ContextProps = {
    contextMenuPosition: {left: number, top: number},
    selectedObject: Frameish,
    threeScene: ThreeScene,
}

export function ObjectContextMenu({
    contextMenuPosition,
    selectedObject,
    threeScene,
}: ContextProps) {
    const { left, top } = contextMenuPosition;
    return (
        <div
            className={styles.contextMenu}
            // ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={() => {
                    threeScene?.duplicateObject(selectedObject!);
                }}
                className={`${styles.duplicate} ${styles.button}`}>
                Duplicate
            </button>
            <button
                onClick={() => {
                    threeScene?.deleteObject(selectedObject!);
                }}
                className={`${styles.delete} ${styles.button}`}>
                Delete
            </button>
        </div>
    );
}
