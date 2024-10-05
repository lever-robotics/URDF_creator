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
    selectedObject: Visual | Collision,
    threeScene: ThreeScene,
}


function ScaleParameters({ selectedObject, threeScene }: ScaleParametersProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState<ParameterValue>(selectedObject.scale.x);
    const [tempY, setTempY] = useState<ParameterValue>(selectedObject.scale.y);
    const [tempZ, setTempZ] = useState<ParameterValue>(selectedObject.scale.z);
    const [tempRadius, setTempRadius] = useState<ParameterValue>(selectedObject.scale.x / 2);
    const [tempHeight, setTempHeight] = useState<ParameterValue>(selectedObject.scale.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        // debugger;
        setTempX(selectedObject.scale.x);
        setTempY(selectedObject.scale.y);
        setTempZ(selectedObject.scale.z);
        setTempRadius(selectedObject.scale.x / 2);
        setTempHeight(selectedObject.scale.z);
    }, [JSON.stringify(selectedObject.scale), threeScene?.toolMode]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setTempX(selectedObject.scale.x);
            setTempY(selectedObject.scale.y);
            setTempZ(selectedObject.scale.z);
            setTempRadius(selectedObject.scale.x / 2);
            setTempHeight(selectedObject.scale.z);
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
        if(validValue === false) return;
        
        const newScale = selectedObject.scale.toArray();
        switch (axis) {
            case "radius":
                validValue = validValue * 2;
            case "scale factor":
            case "x":
                newScale[0] = validValue;
                break;
            case "y":
                newScale[1] = validValue; 
                break;
            case "z":
            case "height":
                newScale[2] = validValue;
                break;
        }
        selectedObject.scale.set(...newScale);
        handleScaleChange(e);
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
            {determineParametersFromShape((selectedObject as Visual | Collision))}
        </Section>
    );
}

export default ScaleParameters;
