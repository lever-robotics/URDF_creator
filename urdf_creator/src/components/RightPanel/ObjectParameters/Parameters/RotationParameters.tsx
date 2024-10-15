import type React from "react";
import { useEffect, useState } from "react";
import type { ParameterValue } from "../ParameterProps";
import type ParameterProps from "../ParameterProps";
import Parameter, { NumberParameter } from "./Parameter";
import Property from "./Property";
import Section from "./Section";

const radToDeg = (radians: number) => (radians * 180) / Math.PI;
const degToRad = (degrees: number) => (degrees * Math.PI) / 180;

const RotationParameters: React.FC<ParameterProps> = ({
    selectedObject,
    threeScene,
}) => {
    const [update, setUpdate] = useState(0);

    useEffect(() => {
        const updateRotation = () => {
            setUpdate((prev) => prev + 1);
        };

        threeScene.addEventListener("parameters", updateRotation);

        return () => {
            threeScene.removeEventListener("parameters", updateRotation);
        };
    }, [threeScene]);

    const x = radToDeg(selectedObject.rotation.x);
    const y = radToDeg(selectedObject.rotation.y);
    const z = radToDeg(selectedObject.rotation.z);

    const handleBlur = (parameter: string, value: number) => {
        const degrees = degToRad(value);
        switch (parameter) {
            case "x": {
                selectedObject.rotation.x = degrees;
                break;
            }
            case "y": {
                selectedObject.rotation.y = degrees;
                break;
            }
            case "z": {
                selectedObject.rotation.z = degrees;
                break;
            }
        }
        threeScene.forceUpdateCode();
    };
    // TODO make sure radToDeg doesn't ruin stuff
    return (
        <Property name="Rotation">
            <NumberParameter
                title="X:"
                parameter="x"
                kind="number"
                units="°degrees"
                value={x}
                handleBlur={handleBlur}
            />
            <NumberParameter
                title="Y:"
                parameter="y"
                kind="number"
                units="°degrees"
                value={y}
                handleBlur={handleBlur}
            />
            <NumberParameter
                title="Z:"
                parameter="z"
                kind="number"
                units="°degrees"
                value={z}
                handleBlur={handleBlur}
            />
        </Property>
    );
};

export default RotationParameters;
