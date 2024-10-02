import React, { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ItemParameterProps from "../ItemParameterProps";

function RotationParameters({ selectedObject, selectedItem, threeScene }: ItemParameterProps) {
    if (!selectedObject) return;
    const radToDeg = (radians: number) => (radians * 180) / Math.PI;
    const degToRad = (degrees: number) => (degrees * Math.PI) / 180;

    const [tempX, setTempX] = useState(
        radToDeg(selectedItem!.rotation.x).toFixed(2)
    );
    const [tempY, setTempY] = useState(
        radToDeg(selectedItem!.rotation.y).toFixed(2)
    );
    const [tempZ, setTempZ] = useState(
        radToDeg(selectedItem!.rotation.z).toFixed(2)
    );

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(radToDeg(selectedItem!.rotation.x).toFixed(2));
        setTempY(radToDeg(selectedItem!.rotation.y).toFixed(2));
        setTempZ(radToDeg(selectedItem!.rotation.z).toFixed(2));
    }, [JSON.stringify(selectedItem!.rotation), threeScene?.toolMode]);

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

    const handleRotationChange = (e: React.ChangeEvent<HTMLInputElement>) => {
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
        }
    };

    const handleRotationBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const newValue = degToRad(parseFloat(checkNegativeZero(e.currentTarget.value)));
        if (isNaN(newValue)) return;
        const newRotation = selectedItem!.objectRotation.toArray();
        switch (axis) {
            case "x":
                newRotation[0] = newValue; 
                break;
            case "y":
                newRotation[1] = newValue; 
                break;
            case "z":
                newRotation[2] = newValue; 
                break;
        }
        selectedItem!.objectRotation.set(...newRotation);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleRotationBlur(e);
        }
    };

    return (
        <Section title="Rotation">
            <ul>
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
            </ul>
        </Section>
    );
}

export default RotationParameters;
