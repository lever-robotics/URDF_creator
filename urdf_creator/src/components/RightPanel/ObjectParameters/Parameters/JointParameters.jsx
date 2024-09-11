import { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";
import Section from "../Section";
import Parameter from "./Parameter";
import OffsetParameters from "./OffsetParameters.jsx";

export default function JointParameters({ selectedObject, stateFunctions }) {
    const [min, setMin] = useState(selectedObject.min);
    const [max, setMax] = useState(selectedObject.max);
    const [maxInput, setMaxInput] = useState(selectedObject.max);
    const [minInput, setMinInput] = useState(selectedObject.min);
    const [jointValue, setJointValue] = useState(selectedObject.jointValue);
    const [jointInput, setJointInput] = useState(selectedObject.jointValue);

    useEffect(() => {
        setMaxInput(selectedObject.max);
        setMinInput(selectedObject.min);
        setMax(selectedObject.max);
        setMin(selectedObject.min);
        setJointInput(selectedObject.jointValue);
        setJointValue(selectedObject.jointValue);
    }, [selectedObject]);

    const handleJointTypeChange = (e) => {
        const value = e.target.value;
        stateFunctions.setJointType(selectedObject, value);
    };

    const toFloat = (value) => {
        let v = parseFloat(value);
        if (isNaN(v)) {
            v = 0;
        }
        return v;
    };

    const checkNegativeZero = (value) => {

        if(value === "-0"){
            console.log("negative 0");
            return 0;
        }else{
            return value;
        }
    }

    const handleJointValueChange = (value) => {
        value = toFloat(value);
        //clamp the value to the min and max
        value = Math.min(Math.max(value, min), max);
        setJointValue(value);
        setJointInput(value);
        stateFunctions.setJointValue(selectedObject, value);
        switch (selectedObject.jointType) {
            case "prismatic":
                stateFunctions.translateAlongJointAxis(selectedObject, value);
                break;
            default:
                stateFunctions.rotateAroundJointAxis(selectedObject, value);
                break;
        }
    };

    const resetJoint = () => {
        handleJointValueChange(0);
    };

    const handleChangeAxisAngle = () => {
        stateFunctions.startRotateJoint(selectedObject);
    };

    const handleChangeAxisOrigin = () => {
        stateFunctions.startMoveJoint(selectedObject);
    };

    const handleMinValueChange = (value) => {
        value = toFloat(checkNegativeZero(value));
        setMinInput(value);
        setMin(value);
        stateFunctions.setJointMinMax(selectedObject, "min", value);
    };

    const handleMaxValueChange = (value) => {
        value = toFloat(checkNegativeZero(value));
        setMaxInput(value);
        setMax(value);
        stateFunctions.setJointMinMax(selectedObject, "max", value);
    };

    const reattachLink = () => {
        stateFunctions.reattachLink(selectedObject);
    };

    return (
        <Section title="Joint Parameters">
            <div>
                <strong>Parent Link:</strong>
                <span> {selectedObject.parentName}</span>
            </div>
            <OffsetParameters selectedObject={selectedObject} stateFunctions={stateFunctions} />
            {!selectedObject.isBaseLink && (
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
                                    size="small"
                                    className="joint-input"
                                    value={minInput}
                                    onChange={(e) => {
                                        setMinInput(e.target.value);
                                    }}
                                    onBlur={(e) => {
                                        handleMinValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleMinValueChange(e.target.value);
                                    }}
                                />
                                <Parameter
                                    title="Max:"
                                    size="small"
                                    className="joint-input"
                                    value={maxInput}
                                    onChange={(e) => {
                                        setMaxInput(e.target.value);
                                    }}
                                    onBlur={(e) => {
                                        handleMaxValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleMaxValueChange(e.target.value);
                                    }}
                                />
                                <Parameter
                                    title="Value:"
                                    size="small"
                                    className="joint-input"
                                    value={jointInput}
                                    onChange={(e) => {
                                        setJointInput(e.target.value);
                                    }}
                                    onBlur={(e) => {
                                        handleJointValueChange(e.target.value);
                                    }}
                                    onKeyPress={(e) => {
                                        if (e.key === "Enter") handleJointValueChange(e.target.value);
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
                                    handleJointValueChange(e.target.value);
                                }}
                                onBlur={stateFunctions.forceUpdateCode}
                            />
                        </>
                    )}
                </>
            )}
        </Section>
    );
}
