import type Frame from "../../../Models/Frame";
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

function VisualParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    const visuals = selectedObject.visuals;

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
