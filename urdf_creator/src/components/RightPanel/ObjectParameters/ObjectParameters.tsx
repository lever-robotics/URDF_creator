import { useEffect, useState } from "react";
import Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import { Selectable } from "../../ThreeDisplay/ThreeScene";
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
    const [selectedObject, setSelectedObject] = useState<Frame | null>();
    const [update, setUpdate] = useState(0);

    useEffect(() => {
        const updateSelected = () => {
            const selected = threeScene.selectedObject;
            if (selected instanceof Frame || !selected) {
                setSelectedObject(selected);
            } else {
                setSelectedObject(selected.frame);
            }
        };
        const update = () => {
            setUpdate((prev) => prev + 1);
        };
        updateSelected();

        threeScene.addEventListener("selectedObject", updateSelected);
        threeScene.addEventListener("parameters", update);

        return () => {
            threeScene.removeEventListener("selectedObject", updateSelected);
            threeScene.removeEventListener("parameters", update);
        };
    }, [threeScene]);

    if (selectedFormat !== "Parameters") return null;

    // If there is no selected Object
    if (!selectedObject) {
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
