import { useState, useEffect } from "react";
import ToggleSection from "../ToggleSection";
import Parameter from "./Parameter";

function ScaleParameters({ selectedObject, stateFunctions }) {
    const [tempX, setTempX] = useState(selectedObject.objectScale.x);
    const [tempY, setTempY] = useState(selectedObject.objectScale.y);
    const [tempZ, setTempZ] = useState(selectedObject.objectScale.z);
    const [tempRadius, setTempRadius] = useState(selectedObject.objectScale.x / 2);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(selectedObject.objectScale.x);
        setTempY(selectedObject.objectScale.y);
        setTempZ(selectedObject.objectScale.z);
        setTempRadius(selectedObject.objectScale.x / 2);
    }, [JSON.stringify(selectedObject.objectScale), stateFunctions.getToolMode()]);

    const handleScaleChange = (e) => {
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
            case "radius":
                setTempRadius(tempValue);
                break;
        }
    };

    const handleScaleBlur = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        let newValue = parseFloat(e.target.value);

        if (axis === "radius") {
            newValue = newValue * 2;
        }

        if (newValue <= 0) {
            newValue = 0.001;
        }

        if (isNaN(newValue)) return;
        stateFunctions.transformObject(selectedObject, "scale", axis, newValue);
    };

    const handleKeyDown = (e) => {
        if (e.key === "Enter") {
            const axis = e.target.title.toLowerCase().replace(":", "");
            let newValue = parseFloat(e.target.value);

            if (axis === "radius") {
                newValue = newValue * 2;
            }

            if (newValue <= 0) {
                newValue = 0.001;
            }

            if (isNaN(newValue)) return;
            stateFunctions.transformObject(selectedObject, "scale", axis, newValue);
        }
    };

    const props = {
        type: "number",
        unit: "m",
        onChange: handleScaleChange,
        onBlur: handleScaleBlur,
        onKeyDown: handleKeyDown
    };

    const determineParametersFromShape = (shape) => {
        switch (shape) {
            case "sphere":
                return (
                    <Parameter
                        title="Radius:"
                        value={tempRadius}
                        {...props}
                    />
                );
            case "cylinder":
                return (
                    <>
                        <Parameter
                            title="Radius:"
                            value={tempRadius}
                            {...props}
                        />
                        <Parameter
                            title="Height:"
                            value={tempZ}
                            {...props}
                        />
                    </>
                );
            case "cube":
                return (
                    <>
                        <Parameter
                            title="X:"
                            value={tempX}
                            {...props}
                        />
                        <Parameter
                            title="Y:"
                            value={tempY}
                            {...props}
                        />
                        <Parameter
                            title="Z:"
                            value={tempZ}
                            {...props}
                        />
                    </>
                );
            default:
                throw Error("Shape not supported");
        }
    };

    return (
        <ToggleSection title="Scale" open={stateFunctions.getToolMode() === "scale"}>
            <ul>{determineParametersFromShape(selectedObject.shape)}</ul>
        </ToggleSection>
    );
}

export default ScaleParameters;
