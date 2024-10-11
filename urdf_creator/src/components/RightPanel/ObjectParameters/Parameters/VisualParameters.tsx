import Frame from "../../../../Models/Frame";
import Inertia from "../../../../Models/Inertia";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import type { Selectable } from "../../../ThreeDisplay/ThreeScene";
import styles from "../ObjectParameters.module.css";
import MaterialParameters from "./MaterialParameters";
import MeshParameters from "./MeshParameters";
import PositionParameters from "./PositionParameters";
import Property from "./Property";
import RotationParameters from "./RotationParameters";
import ScaleParameters from "./ScaleParameters";
import Section from "./Section";

function VisualParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Selectable;
}) {
    if (
        selectedObject instanceof Collision ||
        selectedObject instanceof Inertia
    )
        return null;

    const visuals =
        selectedObject instanceof Frame
            ? selectedObject.visuals
            : selectedObject.frame.visuals;

    return (
        <Section title="Visuals">
            {visuals.map((visual, index) => (
                <Property key={visual.name} name={visual.name}>
                    <MaterialParameters
                        selectedObject={visual}
                        threeScene={threeScene}
                    />
                    <PositionParameters
                        selectedObject={visual}
                        threeScene={threeScene}
                    />
                    <RotationParameters
                        selectedObject={visual}
                        threeScene={threeScene}
                    />
                    <ScaleParameters
                        selectedObject={visual}
                        threeScene={threeScene}
                    />
                    <MeshParameters
                        selectedObject={visual}
                        threeScene={threeScene}
                    />
                </Property>
            ))}
        </Section>
    );
}

export default VisualParameters;
