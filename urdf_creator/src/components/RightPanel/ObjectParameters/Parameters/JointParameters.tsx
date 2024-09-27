import React, { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";
import Section from "../Section";
import Parameter from "./Parameter";
import OffsetParameters from "./OffsetParameters";
import ParameterProps from "../ParameterProps";

export default function JointParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    const [min, setMin] = useState(selectedObject.min);
    const [max, setMax] = useState(selectedObject.max);
    const [maxInput, setMaxInput] = useState(selectedObject.max);
    const [minInput, setMinInput] = useState(selectedObject.min);
    const [jointValue, setJointValue] = useState(selectedObject.jointValue);
    const [jointInput, setJointInput] = useState(selectedObject.jointValue.toString());

    useEffect(() => {
        setMaxInput(selectedObject.max);
        setMinInput(selectedObject.min);
        setMax(selectedObject.max);
        setMin(selectedObject.min);
        setJointInput(selectedObject.jointValue.toString());
        setJointValue(selectedObject.jointValue);
    }, [selectedObject]);

    const handleJointTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        const value = e.currentTarget.value;
        selectedObject.jointType = value;
        threeScene.forceUpdateBoth();
    };

    const toFloat = (value: string) => {
        let v = parseFloat(value);
        if (isNaN(v)) {
            v = 0;
        }
        return v;
    };

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

    const handleJointValueChange = (valueString: string) => {
        let value = toFloat(valueString);
        //clamp the value to the min and max
        value = Math.min(Math.max(value, min), max);
        setJointValue(value);
        setJointInput(value.toString());
        selectedObject.jointValue = value;
        switch (selectedObject.jointType) {
            case "prismatic":
                threeScene.translateAlongJointAxis(selectedObject, value);
                break;
            default:
                threeScene.rotateAroundJointAxis(selectedObject, value);
                break;
        }
    };

    const resetJoint = () => {
        handleJointValueChange("0");
    };

    const handleChangeAxisAngle = () => {
        threeScene.startRotateJoint(selectedObject);
    };

    const handleChangeAxisOrigin = () => {
        threeScene.startMoveJoint(selectedObject);
    };

    const handleMinValueChange = (valueString: string) => {
        const value = toFloat(checkNegativeZero(valueString));
        setMinInput(value);
        setMin(value);
        selectedObject.min = -value;
        threeScene.forceUpdateBoth();
    };

    const handleMaxValueChange = (valueString: string) => {
        const value = toFloat(checkNegativeZero(valueString));
        setMaxInput(value);
        setMax(value);
        selectedObject.max = value;
        threeScene.forceUpdateBoth();
    };

    const reattachLink = () => {
        threeScene.reattachLink(selectedObject);
    };

    return (
        <Section title="Joint Parameters">
            <div>
                <strong>Parent Link:</strong>
                <span> {selectedObject.parentName}</span>
            </div>
            <OffsetParameters selectedObject={selectedObject} threeScene={threeScene} />
            {!selectedObject.isRootFrame && (
                <>
                    <div>
                        <strong>Joint Type:</strong>
                        <select value={selectedObject.jointType} onChange={handleJointTypeChange}>
                            <option value="fixed">Fixed</option>
                            <option value="revolute">Revolute</option>
                            <option value="continuous">Continuous</option>
                            <option value="prismatic">Prismatic</option>
                            {/* <option value="planar">Planar</option>
                        <option value="floating">Floating</option> */}
                        </select>
                    </div>
                    {selectedObject.jointType !== "fixed" && (
                        <>
                            <button onClick={handleChangeAxisAngle} onBlur={reattachLink}>
                                Change Axis Angle
                            </button>
                            <button onClick={handleChangeAxisOrigin} onBlur={reattachLink}>
                                Change Axis Origin
                            </button>
                            <ul>
                                <Parameter
                                    title="Min:"
                                    className="joint-input"
                                    value={minInput}
                                    onChange={(e) => {
                                        setMinInput(Number(e.currentTarget.value));
                                    }}
                                    onBlur={(e) => {
                                        handleMinValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleMinValueChange(e.currentTarget.value);
                                    }}
                                />
                                <Parameter
                                    title="Max:"
                                    className="joint-input"
                                    value={maxInput}
                                    onChange={(e) => {
                                        setMaxInput(Number(e.target.value));
                                    }}
                                    onBlur={(e) => {
                                        handleMaxValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleMaxValueChange(e.currentTarget.value);
                                    }}
                                />
                                <Parameter
                                    title="Value:"
                                    className="joint-input"
                                    value={jointInput}
                                    onChange={(e) => {
                                        setJointInput(e.target.value);
                                    }}
                                    onBlur={(e) => {
                                        handleJointValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleJointValueChange(e.currentTarget.value);
                                    }}
                                />
                            </ul>
                            <button onClick={resetJoint}>Reset</button>
                            <Slider
                                value={jointValue}
                                step={0.01}
                                min={min}
                                max={max}
                                aria-label="Default"
                                valueLabelDisplay="auto"
                                onChange={(e) => {
                                    handleJointValueChange((e.target! as HTMLInputElement).value);
                                }}
                                onBlur={threeScene.forceUpdateCode}
                            />
                        </>
                    )}
                </>
            )}
        </Section>
    );
}
