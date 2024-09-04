import { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";

function OffsetParameters({ selectedObject, stateFunctions }) {
    const [tempX, setTempX] = useState(selectedObject.offset.x);
    const [tempY, setTempY] = useState(selectedObject.offset.y);
    const [tempZ, setTempZ] = useState(selectedObject.offset.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(selectedObject.offset.x);
        setTempY(selectedObject.offset.y);
        setTempZ(selectedObject.offset.z);

    }, [JSON.stringify(selectedObject.offset)]);

    const handleOffsetChange = (e) => {
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

    const handleOffsetBlur = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(e.target.value);
        if (isNaN(newValue)) return;
        stateFunctions.transformObject(
            selectedObject,
            "offset",
            axis,
            newValue
        );
    };

    const handleKeyDown = (e) => {
        if(e.key === "Enter"){
            const axis = e.target.title.toLowerCase().replace(":", "");
            const newValue = parseFloat(e.target.value);
            if (isNaN(newValue)) return;
            stateFunctions.transformObject(
                selectedObject,
                "offset",
                axis,
                newValue
            );
        }
    }

    return (
        <Section title="Offset">
            <ul>
                <Parameter
                    title="X:"
                    type="number"
                    units="m"
                    value={tempX}
                    onChange={handleOffsetChange}
                    onBlur={handleOffsetBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Y:"
                    type="number"
                    units="m"
                    value={tempY}
                    onChange={handleOffsetChange}
                    onBlur={handleOffsetBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Z:"
                    type="number"
                    units="m"
                    value={tempZ}
                    onChange={handleOffsetChange}
                    onBlur={handleOffsetBlur}
                    onKeyDown={handleKeyDown}
                />
            </ul>
        </Section>
    );
}

export default OffsetParameters;
