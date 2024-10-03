import React, { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";
import Section from "../Section";
import Parameter from "./Parameter";
import OffsetParameters from "./OffsetParameters";
import ParameterProps, { ParameterValue } from "../ParameterProps";
import styles from "../ObjectParameters.module.css";

export default function JointParameters({ selectedObject, threeScene }: ParameterProps) {
    if (!selectedObject) return;
    const [min, setMin] = useState<ParameterValue>(selectedObject.min);
    const [max, setMax] = useState<ParameterValue>(selectedObject.max);
    const [jointValue, setJointValue] = useState(selectedObject.jointValue);
    const [jointInput, setJointInput] = useState(selectedObject.jointValue.toString());

    useEffect(() => {
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

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = parseFloat(value);
        if(isNaN(newValue)){
            setMin(selectedObject.min);
            setMax(selectedObject.max);
            return false;
        }

        if(Object.is(newValue, -0)){
            return 0;
        }else{
            return newValue;
        }
    }

    const handleJointValueChange = (valueString: string) => {
        let value = toFloat(valueString);
        //clamp the value to the min and max
        value = Math.min(Math.max(value, min as number), max as number);
        setJointValue(value);
        setJointInput(value.toString());
        selectedObject.jointValue = value;
        switch (selectedObject.jointType) {
            case "prismatic":
                selectedObject.translateAlongJointAxis(value);
                break;
            default:
                selectedObject.rotateAroundJointAxis(value);
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

    const handleMinChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const value = e.target.value;
        setMin(value);
    };

    const handleMaxChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const value = e.target.value;
        setMax(value);
    };

    const handleMinMaxBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const type = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if(validValue === false) return;
        switch (type) {
            case "min":
                selectedObject.min = validValue;
                setMin(validValue); 
                break;
            case "max":
                selectedObject.max = validValue;
                setMax(validValue); 
                break;
        }
    }

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleMinMaxBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    }

    const reattachLink = () => {
        threeScene.reattachLink(selectedObject);
    };

    return (
        <div className={styles.basicParams}>
            <div className={styles.basic}>
                Joint
                <div className={styles.jointParent}>
                    Parent Link:
                    {selectedObject.parentName}
                </div>
            </div>
            <OffsetParameters selectedObject={selectedObject} threeScene={threeScene} />
            <div className={styles.basic}>
                {!selectedObject.isRootFrame && (
                    <>
                        <div>
                            Joint Type:
                            <select value={selectedObject.jointType} onChange={handleJointTypeChange} className={styles.select}>
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
                                <button className={styles.button} onClick={handleChangeAxisAngle} onBlur={reattachLink}>
                                    Change Axis Angle
                                </button>
                                <button className={styles.button} onClick={handleChangeAxisOrigin} onBlur={reattachLink}>
                                    Change Axis Origin
                                </button>
                                <ul>
                                    <Parameter
                                        title="Min:"
                                        value={min}
                                        onChange={handleMinChange}
                                        onBlur={handleMinMaxBlur}
                                        onKeyDown={handleKeyDown}
                                    />
                                    <Parameter
                                        title="Max:"
                                        value={max}
                                        onChange={handleMaxChange}
                                        onBlur={handleMinMaxBlur}
                                        onKeyDown={handleKeyDown}
                                    />
                                    <Parameter
                                        title="Value:"
                                        value={jointInput}
                                        onChange={(e) => {
                                            setJointInput(e.target.value);
                                        }}
                                        onBlur={(e) => {
                                            handleJointValueChange(e.target.value);
                                        }}
                                        onKeyDown={(e) => {
                                            if (e.key === "Enter") handleJointValueChange(e.currentTarget.value);
                                        }}
                                    />
                                </ul>
                                <button className={styles.button} onClick={resetJoint}>Reset</button>
                                <Slider
                                    value={jointValue}
                                    step={0.01}
                                    min={min as number}
                                    max={max as number}
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
            </div>
        </div>
    );
}
