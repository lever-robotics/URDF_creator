import styles from "./ObjectParameters.module.css";
import ThreeScene, { Selectable } from "../../ThreeDisplay/ThreeScene";
import { Collision, Visual } from "../../../Models/VisualCollision";
import Inertia from "../../../Models/Inertia";
import Frame from "../../../Models/Frame";
import MaterialParameters from "./Parameters/MaterialParameters";
import PositionParameters from "./Parameters/PositionParameters";
import RotationParameters from "./Parameters/RotationParameters";
import ScaleParameters from "./Parameters/ScaleParameters";
import MeshParameters from "./Parameters/MeshParameters";
import Section from "./Parameters/Section";
import Property from "./Parameters/Property";

function CollisionParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Selectable;
}) {
    if((selectedObject instanceof Visual) || (selectedObject instanceof Inertia)) return null;
    
    const collisions = selectedObject instanceof Frame ? selectedObject.collisions : selectedObject.frame.collisions;

    return (
        <Section title={"Collisions"}>
            {collisions.map((collision, index) => (
                <Property key={index} name={collision.name}>
                    <MaterialParameters
                        selectedObject={collision}
                        threeScene={threeScene}
                    />
                    <PositionParameters
                        selectedObject={collision}
                        selectedItem={collision}
                        threeScene={threeScene}
                    />
                    <RotationParameters
                        selectedObject={collision}
                        selectedItem={collision}
                        threeScene={threeScene}
                    />
                    <ScaleParameters
                        selectedObject={collision}
                        threeScene={threeScene}
                    />
                    <MeshParameters
                        selectedObject={collision}
                        threeScene={threeScene}
                    />
                </Property>
            ))}
        </Section>
    );
}

export default CollisionParameters;