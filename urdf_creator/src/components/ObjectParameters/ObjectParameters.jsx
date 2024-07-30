import ToggleSection from "./ToggleSection";
import BasicParameters from "./Parameters/BasicParameters";
import PositionParameters from "./Parameters/PositionParameters";
import RotationParameters from "./Parameters/RotationParameters";
import ScaleParameters from "./Parameters/ScaleParameters";
import InertiaParameters from "./Parameters/InertiaParameters";
import JointParameters from "./Parameters/JointParameters";
import SensorsParameters from "./Parameters/SensorParameters";
import MeshParameters from "./Parameters/MeshParameters";
import "./ObjectParameters.css";

function ObjectParameters({
    selectedObject,
    transformObject,
    setMass,
    setJointType,
    setInertia,
    setSensor,
    stateFunctions,
    setMesh,
}) {
    if (!selectedObject) {
        return (
            <div className="object-parameters">
                <h3>Object Parameters</h3>
                No object selected
            </div>
        );
    }

    return (
        <div className="object-parameters">
            <h3>Object Parameters</h3>
            <BasicParameters
                stateFunctions={stateFunctions}
                selectedObject={selectedObject}
            />
            <PositionParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
            <RotationParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
            <ScaleParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
            <InertiaParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
            {!selectedObject.isBaseLink && (
                <ToggleSection title="Joint Parameters">
                    <JointParameters
                        selectedObject={selectedObject}
                        stateFunctions={stateFunctions}
                    />
                </ToggleSection>
            )}
            <ToggleSection title="Sensor Parameters">
                <SensorsParameters
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            </ToggleSection>
            <ToggleSection title="Mesh Parameters">
                <MeshParameters
                    selectedObject={selectedObject}
                    setMesh={setMesh}
                />
            </ToggleSection>
        </div>
    );
}

export default ObjectParameters;
