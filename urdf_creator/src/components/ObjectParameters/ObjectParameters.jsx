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

function ObjectParameters({ selectedObject, transformObject, setUserColor, setMass, setJointType, setInertia, setSensor, stateFunctions, setMesh }) {
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
            <BasicParameters selectedObject={selectedObject} setUserColor={setUserColor} />
            <ToggleSection title="Position">
                <PositionParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <ToggleSection title="Rotation">
                <RotationParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <ToggleSection title="Scale">
                <ScaleParameters selectedObject={selectedObject} transformObject={transformObject} />
            </ToggleSection>
            <InertiaParameters selectedObject={selectedObject} setInertia={setInertia} setMass={setMass}/>
            {!selectedObject.isBaseLink && (
                <ToggleSection title="Joint Parameters">
                    <JointParameters selectedObject={selectedObject} setJoint={setJointType} stateFunctions={stateFunctions} />
                </ToggleSection>
            )}
            <ToggleSection title="Sensor Parameters">
                <SensorsParameters selectedObject={selectedObject} setSensor={setSensor} />
            </ToggleSection>
            <ToggleSection title="Mesh Parameters">
                <MeshParameters selectedObject={selectedObject} setMesh={setMesh} />
            </ToggleSection>
        </div>
    );
}

export default ObjectParameters;
