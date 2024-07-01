import ToggleSection from "./ToggleSection";
import BasicParameters from "./Parameters/BasicParameters";
import PositionParameters from "./Parameters/PositionParameters";
import RotationParameters from "./Parameters/RotationParameters";
import ScaleParameters from "./Parameters/ScaleParameters";
import InertiaParameters from "./Parameters/InertiaParameters";
import JointParameters from "./Parameters/JointParameters";
import SensorsParameters from "./Parameters/SensorParameters";
import "./ObjectParameters.css";

function ObjectParameters({ selectedObject, transformObject, setUserColor, setMass, setJointType, setInertia, setSensor, stateFunctions }) {
    if (!selectedObject || !selectedObject.userData) {
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
            <ToggleSection title="Basic Parameters">
                <BasicParameters selectedObject={selectedObject} setUserColor={setUserColor} />
            </ToggleSection>
            <ToggleSection title="Position">
                <PositionParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <ToggleSection title="Rotation">
                <RotationParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <ToggleSection title="Scale">
                <ScaleParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <ToggleSection title="Inertia Parameters">
                <InertiaParameters selectedObject={selectedObject} setInertia={setInertia} setMass={setMass}/>
            </ToggleSection>
            {!selectedObject.userData.isBaseLink && (
                <ToggleSection title="Joint Parameters">
                    <JointParameters selectedObject={selectedObject} setJoint={setJointType} stateFunctions={stateFunctions} />
                </ToggleSection>
            )}
            <ToggleSection title="Sensor Parameters">
                <SensorsParameters selectedObject={selectedObject} setSensor={setSensor} />
            </ToggleSection>
        </div>
    );
}

export default ObjectParameters;
