import Frame, { Frameish } from "../../Models/Frame";
import ThreeScene from "../ThreeDisplay/ThreeScene";
import { ContextMenu } from "./LinkTree";
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
    if(!selectedTreeObject) return;
    const { left, top } = contextMenuPosition;

    const onDelete = () => {
        if(selectedTreeObject instanceof Frame){
            threeScene?.deleteObject(selectedTreeObject);
        }else{
            // threeScene?.duplicateProperty(selectedTreeObject); //TODO
        }
    }

    const onDuplicate = () => {
        if(selectedTreeObject instanceof Frame){
            threeScene?.duplicateObject(selectedTreeObject);
        }else{
            // threeScene?.deleteProperty(selectedTreeObject); //TODO
        }
    }

    return (
        <div
            className={styles.contextMenu}
            // ref={objectContextMenu}
            style={{ left: left, top: top }}>
            <button
                onClick={onDuplicate}
                className={`${styles.duplicate} ${styles.button}`}>
                Duplicate
            </button>
            <button
                onClick={onDelete}
                className={`${styles.delete} ${styles.button}`}>
                Delete
            </button>
        </div>
    );
}
