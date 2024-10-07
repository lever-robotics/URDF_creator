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
        <div>
            {collisions.map((collision, index) => (
                <div
                    key={`collision-${index}`}
                    className={styles.basicParams}>
                    <div className={styles.title}>Collision {index + 1}</div>
                    <MaterialParameters
                        selectedObject={collision}
                        threeScene={threeScene}
                    />
                    <PositionParameters
                        selectedObject={selectedObject}
                        selectedItem={collision}
                        threeScene={threeScene}
                    />
                    <RotationParameters
                        selectedObject={selectedObject}
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
                </div>
            ))}
        </div>
    );
}

export default CollisionParameters;