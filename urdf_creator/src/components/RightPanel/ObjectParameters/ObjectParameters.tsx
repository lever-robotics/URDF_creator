import FrameParameters from "./FrameParameters";
import InertiaParameters from "./InertiaParameters";
import SensorsParameters from "./SensorParameters";
import ThreeScene from "../../ThreeDisplay/ThreeScene";
import { useEffect, useState } from "react";
import styles from "./ObjectParameters.module.css";
import CollisionParameters from "./CollisionParameters";
import VisualParameters from "./Parameters/VisualParameters";
import Frame from "../../../Models/Frame";
import JointParameters from "./JointParameters";

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
    }, [JSON.stringify(threeScene?.selectedObject)]);

    if (
        selectedObject instanceof Frame &&
        selectedObject.name === "world_frame"
    ) {
        return (
            <div className={styles.objectParameters}>
                <div className={styles.basicParams}>
                    <p>No link selected. Add or Select Link.</p>
                </div>
            </div>
        );
    }

    const props = {
        threeScene: threeScene,
        selectedObject: selectedObject,
    };

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
