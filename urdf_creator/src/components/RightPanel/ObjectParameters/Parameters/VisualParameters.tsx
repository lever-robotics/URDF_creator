import styles from "../ObjectParameters.module.css";
import ThreeScene, { Selectable } from "../../../ThreeDisplay/ThreeScene";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import Inertia from "../../../../Models/Inertia";
import Frame from "../../../../Models/Frame";
import MaterialParameters from "./MaterialParameters";
import PositionParameters from "./PositionParameters";
import RotationParameters from "./RotationParameters";
import ScaleParameters from "./ScaleParameters";
import MeshParameters from "./MeshParameters";
import Section from "./Section";
import Property from "./Property";

function VisualParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Selectable;
}) {
    if((selectedObject instanceof Collision) || (selectedObject instanceof Inertia)) return null;
    
    const visuals = selectedObject instanceof Frame ? selectedObject.visuals : selectedObject.frame.visuals;

    return (
        <Section title="Visuals">
            {visuals.map((visual, index) => (
                    <Property key={index} name={visual.name}>
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