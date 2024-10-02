import React, { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import Frame from "../../../../Models/Frame";
import ItemParameterProps from "../ItemParameterProps";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import Inertia from "../../../../Models/Inertia";


function ScaleParameters({ selectedObject, selectedItem, threeScene }: ItemParameterProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState(selectedObject.objectScale.x);
    const [tempY, setTempY] = useState(selectedObject.objectScale.y);
    const [tempZ, setTempZ] = useState(selectedObject.objectScale.z);
    const [tempRadius, setTempRadius] = useState(selectedObject.objectScale.x / 2);
    const [tempHeight, setTempHeight] = useState(selectedObject.objectScale.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        // debugger;
        setTempX(selectedObject.objectScale.x);
        setTempY(selectedObject.objectScale.y);
        setTempZ(selectedObject.objectScale.z);
        setTempRadius(selectedObject.objectScale.x / 2);
        setTempHeight(selectedObject.objectScale.z);
    }, [JSON.stringify(selectedObject.objectScale), threeScene?.toolMode]);

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

    const handleScaleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const tempValue = Number(e.target.value);
        switch (axis) {
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
        let newValue = parseFloat(checkNegativeZero(e.currentTarget.value));

        if (newValue <= 0) {
            newValue = 0.001;
        }

        if (isNaN(newValue)) return;
        
        const newScale = selectedObject.objectScale.toArray();
        switch (axis) {
            case "radius":
                newValue = newValue * 2;
            case "x":
                newScale[0] = newValue;
                break;
            case "y":
                newScale[1] = newValue; 
                break;
            case "z":
            case "height":
                newScale[2] = newValue;
                break;
        }
        selectedObject.objectScale.set(...newScale);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleScaleBlur(e);
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
            <ul>{determineParametersFromShape((selectedItem as Visual | Collision))}</ul>
        </Section>
    );
}

export default ScaleParameters;
