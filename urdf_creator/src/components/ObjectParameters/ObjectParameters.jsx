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

function ObjectParameters({ selectedObject, stateFunctions }) {
    if (!selectedObject) {
        return (
            <div className="object-parameters">
                <h3>Link Parameters</h3>
                No link selected
            </div>
        );
    }

    return (
        <div className="object-parameters">
            <h3>Link Parameters</h3>
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
                <JointParameters
                    selectedObject={selectedObject}
                    stateFunctions={stateFunctions}
                />
            )}
            <SensorsParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            />
            {/* Disabling STL upload for now */}
            {/* <MeshParameters
                selectedObject={selectedObject}
                stateFunctions={stateFunctions}
            /> */}
        </div>
    );
}

export default ObjectParameters;
