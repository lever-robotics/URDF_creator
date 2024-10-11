import Frame from "../../../Models/Frame";
import Inertia from "../../../Models/Inertia";
import { Collision, Visual } from "../../../Models/VisualCollision";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import type { Selectable } from "../../ThreeDisplay/ThreeScene";
import styles from "./ObjectParameters.module.css";
import MaterialParameters from "./Parameters/MaterialParameters";
import MeshParameters from "./Parameters/MeshParameters";
import PositionParameters from "./Parameters/PositionParameters";
import Property from "./Parameters/Property";
import RotationParameters from "./Parameters/RotationParameters";
import ScaleParameters from "./Parameters/ScaleParameters";
import Section from "./Parameters/Section";

function CollisionParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Selectable;
}) {
    if (selectedObject instanceof Visual || selectedObject instanceof Inertia)
        return null;

    const collisions =
        selectedObject instanceof Frame
            ? selectedObject.collisions
            : selectedObject.frame.collisions;

    return (
        <Section title={"Collisions"}>
            {collisions.map((collision, index) => (
                <Property key={collision.name} name={collision.name}>
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
