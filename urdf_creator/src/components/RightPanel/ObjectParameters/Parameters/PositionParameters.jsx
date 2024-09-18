import { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";

function PositionParameters({ selectedObject, stateFunctions }) {
    const [tempX, setTempX] = useState(selectedObject.position.x);
    const [tempY, setTempY] = useState(selectedObject.position.y);
    const [tempZ, setTempZ] = useState(selectedObject.position.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(selectedObject.position.x);
        setTempY(selectedObject.position.y);
        setTempZ(selectedObject.position.z);

    }, [JSON.stringify(selectedObject.position)]);

    const checkNegativeZero = (value) => {

        if(value === "-0"){
            console.log("negative 0");
            return 0;
        }else{
            return value;
        }
    }

    const handlePositionChange = (e) => {
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
            default:
                break;
        }
    };

    const handlePositionBlur = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(checkNegativeZero(e.target.value));
        if (isNaN(newValue)) return;
        stateFunctions.transformObject(
            selectedObject,
            "position",
            axis,
            newValue
        );
    };

    const handleKeyDown = (e) => {
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
