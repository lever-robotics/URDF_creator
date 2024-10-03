import React, { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ItemParameterProps from "../ItemParameterProps";
import { ParameterValue } from "../ParameterProps";

function RotationParameters({ selectedObject, selectedItem, threeScene }: ItemParameterProps) {
    if (!selectedObject) return;
    const radToDeg = (radians: number) => (radians * 180) / Math.PI;
    const degToRad = (degrees: number) => (degrees * Math.PI) / 180;

    const [tempX, setTempX] = useState<ParameterValue>(
        radToDeg(selectedItem!.rotation.x).toFixed(2)
    );
    const [tempY, setTempY] = useState<ParameterValue>(
        radToDeg(selectedItem!.rotation.y).toFixed(2)
    );
    const [tempZ, setTempZ] = useState<ParameterValue>(
        radToDeg(selectedItem!.rotation.z).toFixed(2)
    );

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(radToDeg(selectedItem!.rotation.x).toFixed(2));
        setTempY(radToDeg(selectedItem!.rotation.y).toFixed(2));
        setTempZ(radToDeg(selectedItem!.rotation.z).toFixed(2));
    }, [JSON.stringify(selectedItem!.rotation), threeScene?.toolMode]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setTempX(selectedItem?.rotation.x);
            setTempY(selectedItem?.rotation.y);
            setTempZ(selectedItem?.rotation.z);
            return false;
        }

        if(Object.is(newValue, -0)){
            return 0;
        }else{
            return newValue;
        }
    }

    const handleRotationChange = (e: React.ChangeEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const tempValue = e.currentTarget.value;
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
        }
    };

    const handleRotationBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if(validValue === false) return;
        const newRotation = selectedItem!.objectRotation.toArray();
        switch (axis) {
            case "x":
                newRotation[0] = validValue; 
                break;
            case "y":
                newRotation[1] = validValue; 
                break;
            case "z":
                newRotation[2] = validValue; 
                break;
        }
        selectedItem!.objectRotation.set(...newRotation);
        handleRotationChange(e);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleRotationBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    return (
        <Section title="Rotation">
                <Parameter
                    title="X:"
                    type="text"
                    units="°degrees"
                    value={tempX}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Y:"
                    type="text"
                    units="°degrees"
                    value={tempY}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Z:"
                    type="text"
                    units="°degrees"
                    value={tempZ}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
        </Section>
    );
}

export default RotationParameters;
