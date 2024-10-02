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
import ThreeScene from "../../ThreeDisplay/ThreeScene";
import { useEffect, useState } from "react";
import { Material } from "three";
import MaterialParameters from "./Parameters/MaterialParameters";

function ObjectParameters({ threeScene, selectedFormat }: { threeScene: ThreeScene, selectedFormat: string}) {
    if(selectedFormat !== "Parameters") return null;

    const [selectedObject, setSelectedObject] = useState(threeScene?.selectedObject);
    const [visuals, setvisuals] = useState(threeScene?.rootFrame?.visuals);
    const [collisions, setcollisions] = useState(threeScene?.rootFrame?.collisions);
    const [selectedItem, setSelectedItem] = useState(threeScene?.selectedItem);     //If the selected Type is visual, collision, inertia or joint only display that information otherwise display all Only defaulting to display all


    useEffect(() => {
        setSelectedObject(threeScene?.selectedObject);
        setvisuals(threeScene?.rootFrame?.visuals);
        setcollisions(threeScene?.rootFrame?.collisions);
        setSelectedItem(threeScene?.selectedItem);

    }, [JSON.stringify(threeScene?.selectedObject), JSON.stringify(threeScene?.selectedItem)]);
    
    if (!selectedObject) {
        return (
            <div className="object-parameters">
                <h3>Link Parameters</h3>
                No link selected
            </div>
        );
    }

    //put logic here to check selected Item for its different types and display only the relivent information


    return (
    <div className="object-parameters">
        <h3>Link Parameters</h3>
        <BasicParameters
            threeScene={threeScene}
            selectedObject={selectedObject}
        />
        <PositionParameters
                selectedObject={selectedObject}
                selectedItem={selectedObject}
                threeScene={threeScene}
        />
        <RotationParameters
            selectedObject={selectedObject}
            selectedItem={selectedObject}
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
        {visuals && visuals.map((visual, index) => (
            <div key={`visual-${index}`}>
            <h4>Visual {index + 1}</h4>
            <MaterialParameters
                selectedObject={selectedObject}
                selectedItem={visual}
                threeScene={threeScene}
            />
            <PositionParameters
                selectedObject={selectedObject}
                selectedItem={visual}
                threeScene={threeScene}
            />
            <RotationParameters
                selectedObject={selectedObject}
                selectedItem={visual}
                threeScene={threeScene}
            />
            <ScaleParameters
                selectedObject={selectedObject}
                selectedItem={visual}
                threeScene={threeScene}
            />
            <MeshParameters
                selectedObject={selectedObject}
                selectedItem={visual}
                threeScene={threeScene}
            />
            </div>
        ))}
        {collisions && collisions.map((collision, index) => (
            <div key={`collision-${index}`}>
            <h4>Collision {index + 1}</h4>
            <MaterialParameters
                selectedObject={selectedObject}
                selectedItem={collision}
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
                selectedObject={selectedObject}
                selectedItem={collision}
                threeScene={threeScene}
            />
            <MeshParameters
                selectedObject={selectedObject}
                selectedItem={collision}
                threeScene={threeScene}
            />
            </div>
        ))}
    </div>
);
}

export default ObjectParameters;
