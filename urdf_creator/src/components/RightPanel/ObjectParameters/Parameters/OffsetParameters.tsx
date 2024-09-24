import { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ParameterProps from "../ParameterProps";

function OffsetParameters({ selectedObject, stateFunctions }: ParameterProps) {
    if (!selectedObject) return;
    const [tempX, setTempX] = useState(selectedObject.offset.x);
    const [tempY, setTempY] = useState(selectedObject.offset.y);
    const [tempZ, setTempZ] = useState(selectedObject.offset.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(Math.abs(selectedObject.offset.x) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.x.toFixed(4)));
        setTempY(Math.abs(selectedObject.offset.y) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.y.toFixed(4)));
        setTempZ(Math.abs(selectedObject.offset.z) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.z.toFixed(4)));

    }, [JSON.stringify(selectedObject.offset)]);

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

    const handleOffsetChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const tempValue = Number(e.target.value);
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

    const handleOffsetBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(checkNegativeZero(e.currentTarget.value));
        if (isNaN(newValue)) return;
        stateFunctions.transformObject(
            selectedObject,
            "offset",
            axis,
            newValue
        );
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if(e.key === "Enter"){
            handleOffsetBlur(e);
        }
    }

    return (
        <Section title="Offset">
            <ul>
                <Parameter
                    title="X:"
                    type="text"
                    units="m"
                    value={tempX}
                    onChange={handleOffsetChange}
                    onBlur={handleOffsetBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Y:"
                    type="text"
                    units="m"
                    value={tempY}
                    onChange={handleOffsetChange}
                    onBlur={handleOffsetBlur}
                    onKeyDown={handleKeyDown}
                />
                <Parameter
                    title="Z:"
                    type="text"
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
