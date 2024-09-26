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
import { selectClasses } from "@mui/material";
import ParameterProps from "./ParameterProps";
import ThreeScene from "../../ThreeDisplay/ThreeSceneObject";
import { useEffect, useState } from "react";

function ObjectParameters({ threeScene, selectedFormat }: { threeScene: ThreeScene, selectedFormat: string}) {
    if(selectedFormat !== "Parameters") return null;

    const [selectedObject, setSelectedObject] = useState(threeScene?.selectedObject);

    useEffect(() => {
        setSelectedObject(threeScene?.selectedObject);

    }, [JSON.stringify(threeScene?.selectedObject)]);

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
            threeScene={threeScene}
            selectedObject={selectedObject}
        />
        <PositionParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
        />
        <RotationParameters
            selectedObject={selectedObject}
            threeScene={threeScene}
        />
        <ScaleParameters
            selectedObject={selectedObject}
            threeScene={threeScene}
        />
        <InertiaParameters
            selectedObject={selectedObject}
            threeScene={threeScene}
        />
        <JointParameters
            selectedObject={selectedObject}
            threeScene={threeScene}
        />
        <SensorsParameters
            selectedObject={selectedObject}
            threeScene={threeScene}
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
