import ToggleSection from './ToggleSection';
import BasicParameters from './BasicParameters';
import PositionParameters from './PositionParameters';
import RotationParameters from './RotationParameters';
import ScaleParameters from './ScaleParameters';
import InertiaParameters from './InertiaParameters';
import JointParameters from './JointParameters';
import SensorsParameters from './SensorParameters';
import './ObjectParameters.css'

function ObjectParameters({ selectedObject, onUpdate }) {

    if (!selectedObject) {
        return <div></div>;
    }

    return (
        <div className="object-parameters">
            <h3>Object Parameters</h3>
            <ToggleSection title="Basic Parameters">
                <BasicParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
            <ToggleSection title="Position">
                <PositionParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
            <ToggleSection title="Rotation">
                <RotationParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
            <ToggleSection title="Scale">
                <ScaleParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
            <ToggleSection title="Inertia Parameters">
                <InertiaParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
            {!selectedObject.userData.isBaseLink && (
                <ToggleSection title="Joint Parameters">
                    <JointParameters selectedObject={selectedObject} onUpdate={onUpdate} />
                </ToggleSection>
            )}
            <ToggleSection title="Sensor Parameters">
                <SensorsParameters selectedObject={selectedObject} onUpdate={onUpdate} />
            </ToggleSection>
        </div>
    );
}

export default ObjectParameters;
