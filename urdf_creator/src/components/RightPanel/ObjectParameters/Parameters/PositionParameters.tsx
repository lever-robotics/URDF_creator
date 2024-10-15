import type React from "react";
import { useEffect, useRef, useState } from "react";
import type { ParameterValue } from "../ParameterProps";
import type ParameterProps from "../ParameterProps";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

const PositionParameters: React.FC<ParameterProps> = ({
    selectedObject,
    threeScene,
}) => {
    if (!selectedObject) return;
    // const [tempX, setTempX] = useState<ParameterValue>(
    //     selectedObject?.position.x,
    // );
    // const [tempY, setTempY] = useState<ParameterValue>(
    //     selectedObject?.position.y,
    // );
    // const [tempZ, setTempZ] = useState<ParameterValue>(
    //     selectedObject?.position.z,
    // );

    // //implement use effect to update when selected object changes
    // useEffect(() => {
    //     setTempX(selectedObject?.position.x);
    //     setTempY(selectedObject?.position.y);
    //     setTempZ(selectedObject?.position.z);
    // }, [
    //     selectedObject.position.x,
    //     selectedObject.position.y,
    //     selectedObject.position.z,
    // ]);

    // const validateInput = (value: string) => {
    //     // If you click enter or away with invalid input then reset
    //     const newValue = Number.parseFloat(value);
    //     if (Number.isNaN(newValue)) {
    //         setTempX(selectedObject?.position.x);
    //         setTempY(selectedObject?.position.y);
    //         setTempZ(selectedObject?.position.z);
    //         return false;
    //     }

    //     if (Object.is(newValue, -0)) {
    //         return 0;
    //     }

    //     return newValue;
    // };

    // const handlePositionChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    //     const axis = e.target.title.toLowerCase().replace(":", "");
    //     const tempValue = e.target.value;
    //     switch (axis) {
    //         case "x":
    //             setTempX(tempValue);
    //             break;
    //         case "y":
    //             setTempY(tempValue);
    //             break;
    //         case "z":
    //             setTempZ(tempValue);
    //             break;
    //         default:
    //             break;
    //     }
    // };

    // const handlePositionBlur = (
    //     e:
    //         | React.FocusEvent<HTMLInputElement>
    //         | React.KeyboardEvent<HTMLInputElement>,
    // ) => {
    //     const axis = e.currentTarget.title.toLowerCase().replace(":", "");
    //     const validValue = validateInput(e.currentTarget.value);
    //     if (validValue === false) return;
    //     const newPosition = selectedObject.position.toArray();
    //     switch (axis) {
    //         case "x":
    //             newPosition[0] = validValue;
    //             setTempX(selectedObject?.position.x);
    //             break;
    //         case "y":
    //             newPosition[1] = validValue;
    //             setTempY(selectedObject?.position.y);
    //             break;
    //         case "z":
    //             newPosition[2] = validValue;
    //             setTempZ(selectedObject?.position.z);
    //             break;
    //     }
    //     selectedObject.position.set(...newPosition);
    // };

    // const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    //     if (e.key === "Enter") {
    //         handlePositionBlur(e);
    //         (e.target as HTMLInputElement).blur();
    //     }
    // };

    const x = selectedObject.position.x;
    const y = selectedObject.position.y;
    const z = selectedObject.position.z;

    const handleBlur = (parameter: string, value: number) => {
        switch (parameter) {
            case "x": {
                selectedObject.position.setX(value);
                break;
            }
            case "y": {
                selectedObject.position.setY(value);
                break;
            }
            case "z": {
                selectedObject.position.setZ(value);
                break;
            }
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Property name="Position">
            <Parameter
                title="X:"
                kind="number"
                parameter="x"
                value={x}
                handleBlur={handleBlur}
                units="m"
            />
            <Parameter
                title="Y:"
                kind="number"
                parameter="y"
                value={y}
                handleBlur={handleBlur}
                units="m"
            />
            <Parameter
                title="Z:"
                kind="number"
                parameter="z"
                value={z}
                handleBlur={handleBlur}
                units="m"
            />
        </Property>
    );
};

export default PositionParameters;
