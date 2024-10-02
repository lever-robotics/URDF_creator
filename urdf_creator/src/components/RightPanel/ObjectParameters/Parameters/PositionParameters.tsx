import React, { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ItemParameterProps from "../ItemParameterProps";

function PositionParameters({ selectedObject, selectedItem, threeScene }: ItemParameterProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState(selectedItem?.position.x);
    const [tempY, setTempY] = useState(selectedItem?.position.y);
    const [tempZ, setTempZ] = useState(selectedItem?.position.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(selectedItem?.position.x);
        setTempY(selectedItem?.position.y);
        setTempZ(selectedItem?.position.z);

    }, [JSON.stringify(selectedItem?.position)]);

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

    const handlePositionChange = (e: React.ChangeEvent<HTMLInputElement>) => {
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
            default:
                break;
        }
    };

    const handlePositionBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(checkNegativeZero(e.currentTarget.value));
        if (isNaN(newValue)) return;
        const newPosition = selectedItem!.objectPosition.toArray();
        switch (axis) {
            case "x":
                newPosition[0] = newValue; 
                break;
            case "y":
                newPosition[1] = newValue; 
                break;
            case "z":
                newPosition[2] = newValue; 
                break;
        }
        selectedItem!.objectPosition.set(...newPosition);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if(e.key === "Enter"){
            handlePositionBlur(e);
        }
    }

    return (
        <Section title="Position">
            <ul>
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
            </ul>
        </Section>
    );
}

export default PositionParameters;
