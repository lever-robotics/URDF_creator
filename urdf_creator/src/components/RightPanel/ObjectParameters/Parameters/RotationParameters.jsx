import { useState, useEffect } from "react";
import Section from "../Section";
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
    }, [JSON.stringify(selectedObject.rotation), stateFunctions.getToolMode()]);

    const checkNegativeZero = (value) => {

        if(value === "-0"){
            console.log("negative 0");
            return 0;
        }else{
            return value;
        }
    }

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
        const newValue = parseFloat(checkNegativeZero(e.target.value));
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
