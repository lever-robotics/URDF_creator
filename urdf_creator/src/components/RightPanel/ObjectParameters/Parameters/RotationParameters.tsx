import type React from "react";
import { useEffect, useState } from "react";
import type ItemParameterProps from "../ItemParameterProps";
import type { ParameterValue } from "../ParameterProps";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

const radToDeg = (radians: number) => (radians * 180) / Math.PI;
const degToRad = (degrees: number) => (degrees * Math.PI) / 180;

function RotationParameters({
    selectedObject,
    threeScene,
}: ItemParameterProps) {
    if (!selectedObject) return;

    const [tempX, setTempX] = useState<ParameterValue>(
        radToDeg(selectedObject.rotation.x).toFixed(2),
    );
    const [tempY, setTempY] = useState<ParameterValue>(
        radToDeg(selectedObject.rotation.y).toFixed(2),
    );
    const [tempZ, setTempZ] = useState<ParameterValue>(
        radToDeg(selectedObject.rotation.z).toFixed(2),
    );

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(radToDeg(selectedObject.rotation.x).toFixed(2));
        setTempY(radToDeg(selectedObject.rotation.y).toFixed(2));
        setTempZ(radToDeg(selectedObject.rotation.z).toFixed(2));
    }, [
        selectedObject.rotation.x,
        selectedObject.rotation.y,
        selectedObject.rotation.z,
    ]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = Number.parseFloat(value);
        if (Number.isNaN(newValue)) {
            setTempX(selectedObject.rotation.x);
            setTempY(selectedObject.rotation.y);
            setTempZ(selectedObject.rotation.z);
            return false;
        }

        if (Object.is(newValue, -0)) {
            return 0;
        }

        return newValue;
    };

    const handleRotationChange = (
        e:
            | React.ChangeEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>,
    ) => {
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

    const handleRotationBlur = (
        e:
            | React.FocusEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>,
    ) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if (validValue === false) return;
        const newRotation = selectedObject.rotation.toArray();
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
        selectedObject.rotation.set(...newRotation);
        handleRotationChange(e);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleRotationBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    return (
        <Property name="Rotation">
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
        </Property>
    );
}

export default RotationParameters;
