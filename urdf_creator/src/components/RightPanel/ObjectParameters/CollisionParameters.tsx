import type Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
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
    selectedObject: Frame;
}) {
    const collisions = selectedObject.collisions;

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
                        threeScene={threeScene}
                    />
                    <RotationParameters
                        selectedObject={collision}
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
