import Frame, { Frameish } from "../../Models/Frame";
import type ThreeScene from "../ThreeDisplay/ThreeScene";
import type { ContextMenu } from "./LinkTree";
import styles from "./LinkTree.module.css";

type ContextProps = {
    contextMenuPosition: { left: number; top: number };
    selectedTreeObject: ContextMenu;
    threeScene: ThreeScene;
};

export function ObjectContextMenu({
    contextMenuPosition,
    selectedTreeObject,
    threeScene,
}: ContextProps) {
    if (!selectedTreeObject) return;
    const { left, top } = contextMenuPosition;

    const onDelete = () => {
        threeScene?.deleteObject(selectedTreeObject);
    };

    const onDuplicate = () => {
        threeScene?.duplicateObject(selectedTreeObject);
    };

    return (
        <div
            className={styles.contextMenu}
            // ref={objectContextMenu}
            style={{ left: left, top: top }}
        >
            <button
                onClick={onDuplicate}
                className={`${styles.duplicate} ${styles.button}`}
                type="button"
            >
                Duplicate
            </button>
            <button
                onClick={onDelete}
                className={`${styles.delete} ${styles.button}`}
                type="button"
            >
                Delete
            </button>
        </div>
    );
}
