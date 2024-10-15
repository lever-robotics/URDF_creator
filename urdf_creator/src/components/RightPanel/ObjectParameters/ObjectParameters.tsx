import { useEffect, useState } from "react";
import Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import CollisionParameters from "./CollisionParameters";
import FrameParameters, { WorldFrameParameters } from "./FrameParameters";
import InertiaParameters from "./InertiaParameters";
import JointParameters from "./JointParameters";
import styles from "./ObjectParameters.module.css";
import SensorsParameters from "./SensorParameters";
import VisualParameters from "./VisualParameters";

function ObjectParameters({
    threeScene,
    selectedFormat,
}: {
    threeScene: ThreeScene;
    selectedFormat: string;
}) {
    const selected =
        threeScene.selectedObject instanceof Frame ||
        threeScene.selectedObject === null
            ? threeScene.selectedObject
            : threeScene.selectedObject.frame;
    const [selectedObject, setSelectedObject] = useState(selected);

    useEffect(() => {
        const selected =
            threeScene.selectedObject instanceof Frame ||
            threeScene.selectedObject === null
                ? threeScene.selectedObject
                : threeScene.selectedObject.frame;
        setSelectedObject(selected);
    }, [threeScene.selectedObject]);

    if (selectedFormat !== "Parameters") return null;
    if (!threeScene) return null;

    // If there is no selected Object
    if (selectedObject === null) {
        return (
            <div className={styles.objectParameters}>No Object Selected</div>
        );
    }

    const props = {
        threeScene: threeScene,
        selectedObject: selectedObject,
    };

    //check if the selected object is the origin
    if (selectedObject instanceof Frame && selectedObject.isWorldFrame) {
        return (
            <div className={styles.objectParameters}>
                <div className={styles.basicParams}>
                    <WorldFrameParameters {...props} />
                </div>
            </div>
        );
    }

    return (
        <div className={styles.objectParameters}>
            <FrameParameters {...props} />
            <JointParameters {...props} />
            <InertiaParameters {...props} />
            <SensorsParameters {...props} />
            <VisualParameters {...props} />
            <CollisionParameters {...props} />
        </div>
    );
}

export default ObjectParameters;
