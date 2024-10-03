import ThreeScene from "../ThreeDisplay/ThreeScene";
import styles from "./Insert.module.css";

function InsertTool({ threeScene }: { threeScene: ThreeScene }) {
    const addObject = threeScene?.addObject;

    return (
        <>
            <div className={styles.toolbar}>
                <button className={styles.toolbarButton}>
                    Add Link
                </button>
            </div>
            <div className={styles.insertTool}>
                <button className={styles.button} onClick={() => addObject("cube")}>
                    Add Cube
                </button>
                <button
                    className={styles.button}
                    onClick={() => addObject("sphere")}>
                    Add Sphere
                </button>
                <button
                    className={styles.button}
                    onClick={() => addObject("cylinder")}>
                    Add Cylinder
                </button>
            </div>
        </>
    );
}

export default InsertTool;
