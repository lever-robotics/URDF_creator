import React, { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import Frame, { Frameish } from "../../../../Models/Frame";
import ItemParameterProps from "../ItemParameterProps";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import Inertia from "../../../../Models/Inertia";
import ThreeScene from "../../../ThreeDisplay/ThreeScene";
import { ParameterValue } from "../ParameterProps";

type ScaleParametersProps = {
    selectedObject?: Frameish,
    selectedItem?: Visual | Collision,
    threeScene: ThreeScene,
}


function ScaleParameters({ selectedObject, selectedItem, threeScene }: ScaleParametersProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState<ParameterValue>(selectedItem!.objectScale.x);
    const [tempY, setTempY] = useState<ParameterValue>(selectedItem!.objectScale.y);
    const [tempZ, setTempZ] = useState<ParameterValue>(selectedItem!.objectScale.z);
    const [tempRadius, setTempRadius] = useState<ParameterValue>(selectedItem!.objectScale.x / 2);
    const [tempHeight, setTempHeight] = useState<ParameterValue>(selectedItem!.objectScale.z);
    const [tempScaleFactor, setTempScaleFactor] = useState<ParameterValue>(selectedItem!.objectScale.x);

    //implement use effect to update when selected object changes
    useEffect(() => {
        // debugger;
        setTempX(selectedItem!.objectScale.x);
        setTempY(selectedItem!.objectScale.y);
        setTempZ(selectedItem!.objectScale.z);
        setTempRadius(selectedItem!.objectScale.x / 2);
        setTempHeight(selectedItem!.objectScale.z);
        setTempScaleFactor(selectedItem!.objectScale.x);
    }, [JSON.stringify(selectedItem!.objectScale), threeScene?.toolMode]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setTempX(selectedItem?.objectScale.x);
            setTempY(selectedItem?.objectScale.y);
            setTempZ(selectedItem?.objectScale.z);
            setTempRadius(selectedItem!.objectScale.x / 2);
            setTempHeight(selectedItem!.objectScale.z);
            setTempScaleFactor(selectedItem!.objectScale.x);
            return false;
        }

        if (newValue <= 0) {
            return 0.001;
        }else if (Object.is(newValue, -0)){
            return 0;
        }else {
            return newValue;
        }
    }

    const handleScaleChange = (e: React.ChangeEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const tempValue = e.currentTarget.value;
        switch (axis) {
            case "scale factor":
            case "x":
                setTempX(tempValue);
                break;
            case "y":
                setTempY(tempValue);
                break;
            case "z":
                setTempZ(tempValue);
                break;
            case "radius":
                setTempRadius(tempValue);
                break;
            case "height":
                setTempHeight(tempValue);
                break;
        }
    };

    const handleScaleBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        let validValue = validateInput(e.currentTarget.value);
        if (validValue === false) return;
    
        const newScale = selectedItem!.objectScale.toArray();
        debugger;
        // Handle scaling based on object shape
        switch (selectedItem!.shape) {
            case "cube":
                // For a cube, scale x, y, and z independently
                switch (axis) {
                    case "x":
                        newScale[0] = validValue;
                        break;
                    case "y":
                        newScale[1] = validValue;
                        break;
                    case "z":
                        newScale[2] = validValue;
                        break;
                }
                break;
    
            case "cylinder":
                // For a cylinder, radius affects x and y, height affects z
                if (axis === "radius") {
                    // Scale x and y by the radius
                    newScale[0] = validValue * 2; // x-axis
                    newScale[1] = validValue * 2; // y-axis
                } else if (axis === "height") {
                    newScale[2] = validValue; // z-axis
                }
                break;
    
            case "sphere":
                // For a sphere, scale x, y, and z by 2 times the value
                newScale[0] = validValue * 2;
                newScale[1] = validValue * 2;
                newScale[2] = validValue * 2;
                break;
    
            case "mesh":
                // For a mesh, apply uniform scaling along all axes
                newScale[0] = validValue;
                newScale[1] = validValue;
                newScale[2] = validValue;
                break;
    
            default:
                throw new Error(`Unsupported shape: ${selectedItem!.shape}`);
        }
    
        // Apply the new scale to the object
        selectedItem!.objectScale.set(...newScale);
        handleScaleChange(e); // Ensure any other necessary updates happen
    };
    

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleScaleBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    const props = {
        type: "text",
        units: "m",
        onChange: handleScaleChange,
        onBlur: handleScaleBlur,
        onKeyDown: handleKeyDown,
    };

    const determineParametersFromShape = (object: Visual | Collision) => {
        switch (object.shape) {
            case "sphere":
                return <Parameter title="Radius:" value={tempRadius} {...props} />;
            case "mesh":
                return <Parameter title="Scale Factor:" value={tempX} {...props} />;
            case "cylinder":
                return (
                    <>
                        <Parameter title="Radius:" value={tempRadius} {...props} />
                        <Parameter title="Height:" value={tempHeight} {...props} />
                    </>
                );
            case "cube":
                return (
                    <>
                        <Parameter title="X:" value={tempX} {...props} />
                        <Parameter title="Y:" value={tempY} {...props} />
                        <Parameter title="Z:" value={tempZ} {...props} />
                    </>
                );
            default:
                throw Error("Shape not supported: " + object.shape);
        }
    };

    return (
        <Section title="Scale">
            {determineParametersFromShape((selectedItem as Visual | Collision))}
        </Section>
    );
}

export default ScaleParameters;
