import { useState, useEffect } from "react";
import ToggleSection from "../ToggleSection";
import Parameter from "./Parameter";

function RotationParameters({ selectedObject, stateFunctions }) {
    const radToDeg = (radians) => (radians * 180) / Math.PI;
    const degToRad = (degrees) => (degrees * Math.PI) / 180;

    const [tempX, setTempX] = useState(
        radToDeg(selectedObject.rotation.x).toFixed(2)
    );
    const [tempY, setTempY] = useState(
        radToDeg(selectedObject.rotation.y).toFixed(2)
    );
    const [tempZ, setTempZ] = useState(
        radToDeg(selectedObject.rotation.z).toFixed(2)
    );

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(radToDeg(selectedObject.rotation.x).toFixed(2));
        setTempY(radToDeg(selectedObject.rotation.y).toFixed(2));
        setTempZ(radToDeg(selectedObject.rotation.z).toFixed(2));
    }, [JSON.stringify(selectedObject.rotation)]);

    const handleRotationChange = (e) => {
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

    const handleRotationBlur = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(e.target.value);
        if (isNaN(newValue)) return;
        stateFunctions.transformObject(
            selectedObject,
            "rotation",
            axis,
            degToRad(newValue)
        );
    };

    const handleKeyDown = (e) => {
        if (e.key === "Enter") {
            const axis = e.target.title.toLowerCase().replace(":", "");
            const newValue = parseFloat(e.target.value);
            if (isNaN(newValue)) return;
            stateFunctions.transformObject(
                selectedObject,
                "rotation",
                axis,
                degToRad(newValue)
            );
        }
    };

    return (
        <ToggleSection title="Rotation">
            <ul>
                <Parameter
                    title="X:"
                    type="number"
                    units="°degrees"
                    value={tempX}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Y:"
                    type="number"
                    units="°degrees"
                    value={tempY}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Z:"
                    type="number"
                    units="°degrees"
                    value={tempZ}
                    onChange={handleRotationChange}
                    onBlur={handleRotationBlur}
                    onKeyDown={handleKeyDown}
                />
            </ul>
        </ToggleSection>
    );
}

export default RotationParameters;
