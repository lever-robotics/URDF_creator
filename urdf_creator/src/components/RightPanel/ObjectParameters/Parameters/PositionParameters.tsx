import React, { useState, useEffect, useRef } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ItemParameterProps from "../ItemParameterProps";
import { ParameterValue } from "../ParameterProps";

function PositionParameters({ selectedObject, threeScene }: ItemParameterProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState<ParameterValue>(selectedObject?.position.x);
    const [tempY, setTempY] = useState<ParameterValue>(selectedObject?.position.y);
    const [tempZ, setTempZ] = useState<ParameterValue>(selectedObject?.position.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(selectedObject?.position.x);
        setTempY(selectedObject?.position.y);
        setTempZ(selectedObject?.position.z);

    }, [JSON.stringify(selectedObject?.position)]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setTempX(selectedObject?.position.x);
            setTempY(selectedObject?.position.y);
            setTempZ(selectedObject?.position.z);
            return false;
        }

        if(Object.is(newValue, -0)){
            return 0;
        }else{
            return newValue;
        }
    }

    const handlePositionChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const tempValue = e.target.value;
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
            default:
                break;
        }
    };

    const handlePositionBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if(validValue === false) return;
        const newPosition = selectedObject!.position.toArray();
        switch (axis) {
            case "x":
                newPosition[0] = validValue;
                setTempX(selectedObject?.position.x); 
                break;
            case "y":
                newPosition[1] = validValue; 
                setTempY(selectedObject?.position.y);
                break;
            case "z":
                newPosition[2] = validValue;
                setTempZ(selectedObject?.position.z); 
                break;
        }
        selectedObject!.position.set(...newPosition);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if(e.key === "Enter"){
            handlePositionBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    }

    return (
        <Section title="Position">
                <Parameter
                    title="X:"
                    type="text"
                    units="m"
                    value={tempX}
                    onChange={handlePositionChange}
                    onBlur={handlePositionBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Y:"
                    type="text"
                    units="m"
                    value={tempY}
                    onChange={handlePositionChange}
                    onBlur={handlePositionBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Z:"
                    type="text"
                    units="m"
                    value={tempZ}
                    onChange={handlePositionChange}
                    onBlur={handlePositionBlur}
                    onKeyDown={handleKeyDown}
                />
        </Section>
    );
}

export default PositionParameters;
