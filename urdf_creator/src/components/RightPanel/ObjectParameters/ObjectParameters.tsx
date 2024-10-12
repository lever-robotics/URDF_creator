import { useEffect, useState } from "react";
import Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import CollisionParameters from "./CollisionParameters";
import FrameParameters from "./FrameParameters";
import InertiaParameters from "./InertiaParameters";
import JointParameters from "./JointParameters";
import styles from "./ObjectParameters.module.css";
import VisualParameters from "./Parameters/VisualParameters";
import SensorsParameters from "./SensorParameters";

function ObjectParameters({
    threeScene,
    selectedFormat,
}: {
    threeScene: ThreeScene;
    selectedFormat: string;
}) {
    if (selectedFormat !== "Parameters") return null;
    if (!threeScene) return null;
    const selected =
        threeScene.selectedObject instanceof Frame
            ? threeScene.selectedObject
            : threeScene.selectedObject.frame;
    const [selectedObject, setSelectedObject] = useState(selected);

    useEffect(() => {
        const selected =
            threeScene.selectedObject instanceof Frame
                ? threeScene.selectedObject
                : threeScene.selectedObject.frame;
        setSelectedObject(selected);
    }, [threeScene.selectedObject]);

    const props = {
        threeScene: threeScene,
        selectedObject: selectedObject,
    };

    //check if the selected object is the origin or that no object is selected
    if (selectedObject instanceof Frame && selectedObject.isWorldFrame) {
        return (
            <div className={styles.objectParameters}>
                <div className={styles.basicParams}>
                    <FrameParameters {...props} />
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
