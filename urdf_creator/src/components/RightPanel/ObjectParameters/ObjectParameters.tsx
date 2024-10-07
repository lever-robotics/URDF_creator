import BasicParameters from "./Parameters/BasicParameters";
import PositionParameters from "./Parameters/PositionParameters";
import RotationParameters from "./Parameters/RotationParameters";
import ScaleParameters from "./Parameters/ScaleParameters";
import InertiaParameters from "./Parameters/InertiaParameters";
import JointParameters from "./Parameters/JointParameters";
import SensorsParameters from "./Parameters/SensorParameters";
import MeshParameters from "./Parameters/MeshParameters";
import ThreeScene from "../../ThreeDisplay/ThreeScene";
import { useEffect, useState } from "react";
import { Material } from "three";
import MaterialParameters from "./Parameters/MaterialParameters";
import styles from "./ObjectParameters.module.css";
import CollisionParameters from "./Parameters/CollisionParameters";
import VisualParameters from "./Parameters/VisualParameters";
import Frame from "../../../Models/Frame";

function ObjectParameters({
    threeScene,
    selectedFormat,
}: {
    threeScene: ThreeScene;
    selectedFormat: string;
}) {
    if (selectedFormat !== "Parameters") return null;
    if (!threeScene) return null;
    const selected = threeScene.selectedObject instanceof Frame ? threeScene.selectedObject : threeScene.selectedObject.frame;
    const [selectedObject, setSelectedObject] = useState(selected);
    // const [visuals, setvisuals] = useState(threeScene?.selectedObject?.visuals);
    // const [collisions, setcollisions] = useState(
    //     threeScene?.selectedObject?.collisions
    // );
    // const [selectedItem, setSelectedItem] = useState(threeScene?.selectedItem); //If the selected Type is visual, collision, inertia or joint only display that information otherwise display all Only defaulting to display all

    useEffect(() => {
        const selected = threeScene.selectedObject instanceof Frame ? threeScene.selectedObject : threeScene.selectedObject.frame;
        setSelectedObject(selected);
        // setvisuals(threeScene?.selectedObject?.visuals);
        // setcollisions(threeScene?.selectedObject?.collisions);
        // setSelectedItem(threeScene?.selectedItem);
    }, [
        JSON.stringify(threeScene?.selectedObject),
        // JSON.stringify(threeScene?.selectedItem),
    ]);

    if (selectedObject instanceof Frame && selectedObject.name === "world_frame") {
        return (
            <div className={styles.objectParameters}>
                <div className={styles.basicParams}>
                    <p>No link selected. Add or Select Link.</p>
                </div>
            </div>
        );
    }

    //put logic here to check selected Item for its different types and display only the relivent information

    return (
        <div className={styles.objectParameters}>
            <div className={styles.basicParams}>
                <BasicParameters
                    threeScene={threeScene}
                    selectedObject={selectedObject}
                />
                <PositionParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
                <RotationParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
            </div>
            <div className={styles.basicParams}>
                <InertiaParameters
                    selectedObject={selectedObject}
                    threeScene={threeScene}
                />
            </div>
            <JointParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
            <SensorsParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
            <VisualParameters selectedObject={selectedObject} threeScene={threeScene}/>
            <CollisionParameters selectedObject={selectedObject} threeScene={threeScene}/>
        </div>
    );
}

export default ObjectParameters;
