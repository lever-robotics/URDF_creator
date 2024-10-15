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
    const [update, setUpdate] = useState(0);

    useEffect(() => {
        const updatePosition = () => {
            setUpdate((prev) => prev + 1);
        };

        threeScene.addEventListener("parameters", updatePosition);

        return () => {
            threeScene.removeEventListener("parameters", updatePosition);
        };
    }, [threeScene]);

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
