import type React from "react";
import { useEffect, useState } from "react";
import Frame, { Frameish } from "../../../../Models/Frame";
import Inertia from "../../../../Models/Inertia";
import type { Collision, Visual } from "../../../../Models/VisualCollision";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import type { ParameterValue } from "../ParameterProps";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

type ScaleParametersProps = {
    selectedObject: Visual | Collision;
    threeScene: ThreeScene;
};

const ScaleParameters: React.FC<ScaleParametersProps> = ({
    selectedObject,
    threeScene,
}) => {
    const [update, setUpdate] = useState(0);

    useEffect(() => {
        const updateScale = () => {
            setUpdate((prev) => prev + 1);
            console.log("scale");
        };

        threeScene.addEventListener("parameters", updateScale);

        return () => {
            threeScene.removeEventListener("parameters", updateScale);
        };
    }, [threeScene]);

    const shape = selectedObject.shape;
    const x = selectedObject.scale.x;
    const y = selectedObject.scale.y;
    const z = selectedObject.scale.z;
    const cylinderRadius = selectedObject.scale.x / 2;
    const height = selectedObject.scale.z;
    const sphereRadius = selectedObject.scale.x / 2;
    const mesh = selectedObject.scale.x;

    const isShape = (isShape: string) => shape === isShape;

    // TODO modify scaleVector to not take negative values;
    const handleBlur = (parameter: string, value: number) => {
        switch (parameter) {
            case "x":
                selectedObject.scale.setX(value);
                break;
            case "y":
                selectedObject.scale.setY(value);
                break;
            case "z":
                selectedObject.scale.setZ(value);
                break;
            case "cylinderRadius":
                selectedObject.scale.setX(value * 2);
                selectedObject.scale.setY(value * 2);
                break;
            case "height":
                selectedObject.scale.setZ(value);
                break;
            case "sphereRadius":
                selectedObject.scale.set(value * 2, value * 2, value * 2);
                break;
            case "mesh":
                selectedObject.scale.set(value, value, value);
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Property name="Scale">
            {isShape("cube") && (
                <>
                    <Parameter
                        title="X:"
                        units="m"
                        parameter="x"
                        value={x}
                        kind="number"
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Y:"
                        units="m"
                        parameter="y"
                        value={y}
                        kind="number"
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Z:"
                        units="m"
                        parameter="z"
                        value={z}
                        kind="number"
                        handleBlur={handleBlur}
                    />
                </>
            )}
            {isShape("cylinder") && (
                <>
                    <Parameter
                        title="Radius:"
                        units="m"
                        parameter="cylinderRadius"
                        value={cylinderRadius}
                        kind="number"
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Height:"
                        units="m"
                        parameter="height"
                        value={height}
                        kind="number"
                        handleBlur={handleBlur}
                    />
                </>
            )}
            {isShape("sphere") && (
                <Parameter
                    title="Radius:"
                    units="m"
                    parameter="sphereRadius"
                    value={sphereRadius}
                    kind="number"
                    handleBlur={handleBlur}
                />
            )}
            {isShape("mesh") && (
                <Parameter
                    title="Scale Factor:"
                    parameter="mesh"
                    value={mesh}
                    kind="number"
                    handleBlur={handleBlur}
                    toFixed={5}
                />
            )}
        </Property>
    );
};

export default ScaleParameters;
