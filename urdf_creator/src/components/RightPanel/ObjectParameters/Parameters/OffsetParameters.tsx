import { useState, useEffect } from "react";
import Section from "../Section";
import Parameter from "./Parameter";
import ParameterProps, { ParameterValue } from "../ParameterProps";

function OffsetParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    
    const [tempX, setTempX] = useState<ParameterValue>(selectedObject.offset.x);
    const [tempY, setTempY] = useState<ParameterValue>(selectedObject.offset.y);
    const [tempZ, setTempZ] = useState<ParameterValue>(selectedObject.offset.z);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempX(Math.abs(selectedObject.offset.x) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.x.toFixed(4)));
        setTempY(Math.abs(selectedObject.offset.y) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.y.toFixed(4)));
        setTempZ(Math.abs(selectedObject.offset.z) < 0.00001 ? 0.0 : parseFloat(selectedObject.offset.z.toFixed(4)));

    }, [JSON.stringify(selectedObject.offset)]);

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setTempX(selectedObject?.position.x);
            setTempY(selectedObject?.position.y);
            setTempZ(selectedObject?.position.z);
            return false;
        }

        if(Object.is(newValue, -0)){
            return 0;
        }else{
            return newValue;
        }
    }

    const handleOffsetChange = (e: React.ChangeEvent<HTMLInputElement>) => {
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

    const handleOffsetBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const axis = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if(!validValue) return;
        const newOffset = selectedObject!.offset.toArray();
        switch (axis) {
            case "x":
                newOffset[0] = validValue;
                setTempX(selectedObject?.offset.x); 
                break;
            case "y":
                newOffset[1] = validValue; 
                setTempY(selectedObject?.offset.y);
                break;
            case "z":
                newOffset[2] = validValue;
                setTempZ(selectedObject?.offset.z); 
                break;
        }
        selectedObject.offset.set(...newOffset);
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if(e.key === "Enter"){
            handleOffsetBlur(e);
            (e.target as HTMLInputElement).blur();
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
