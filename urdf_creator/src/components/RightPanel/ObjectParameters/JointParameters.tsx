import Slider from "@mui/material/Slider";
import type React from "react";
import { useEffect, useState } from "react";
import type Frame from "../../../Models/Frame";
import type { JointType } from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import styles from "./ObjectParameters.module.css";
import ParameterProps, { type ParameterValue } from "./ParameterProps";
import OffsetParameters from "./Parameters/OffsetParameters";
import Parameter from "./Parameters/Parameter";
import Property from "./Parameters/Property";
import Section from "./Parameters/Section";

export default function JointParameters({
    selectedObject,
    threeScene,
}: {
    selectedObject: Frame;
    threeScene: ThreeScene;
}) {
    if (!selectedObject) return;
    const [min, setMin] = useState<ParameterValue>(selectedObject.min);
    const [max, setMax] = useState<ParameterValue>(selectedObject.max);
    const [jointValue, setJointValue] = useState(selectedObject.jointValue);
    const [jointInput, setJointInput] = useState(
        selectedObject.jointValue.toString(),
    );

    useEffect(() => {
        setMax(selectedObject.max);
        setMin(selectedObject.min);
        setJointInput(selectedObject.jointValue.toString());
        setJointValue(selectedObject.jointValue);
    }, [selectedObject]);

    const handleJointTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        const value = e.currentTarget.value;
        selectedObject.jointType = value as JointType;
        threeScene.forceUpdateBoth();
    };

    const toFloat = (value: string) => {
        let v = Number.parseFloat(value);
        if (Number.isNaN(v)) {
            v = 0;
        }
        return v;
    };

    const validateInput = (value: string) => {
        // If you click enter or away with invalid input then reset
        const newValue = Number.parseFloat(value);
        if (Number.isNaN(newValue)) {
            setMin(selectedObject.min);
            setMax(selectedObject.max);
            return false;
        }

        if (Object.is(newValue, -0)) {
            return 0;
        }

        return newValue;
    };

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

    const handleMinMaxBlur = (
        e:
            | React.FocusEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>,
    ) => {
        const type = e.currentTarget.title.toLowerCase().replace(":", "");
        const validValue = validateInput(e.currentTarget.value);
        if (validValue === false) return;
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
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleMinMaxBlur(e);
            (e.target as HTMLInputElement).blur();
        }
    };

    const reattachLink = () => {
        threeScene.reattachLink(selectedObject);
    };

    const determineJointFromType = (jointType: JointType) => {
        if (jointType === "fixed") {
            return <></>;
        }
        return (
            <>
                <Property name="Joint Limits">
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
                </Property>
                <Property>
                    <Parameter
                        title="Slider Value:"
                        value={jointInput}
                        onChange={(e) => {
                            setJointInput(e.target.value);
                        }}
                        onBlur={(e) => {
                            handleJointValueChange(e.target.value);
                        }}
                        onKeyDown={(e) => {
                            if (e.key === "Enter")
                                handleJointValueChange(e.currentTarget.value);
                        }}
                    />
                    <button
                        className={styles.button}
                        onClick={resetJoint}
                        type="button"
                    >
                        Reset Slider Value
                    </button>
                </Property>
                <Slider
                    value={jointValue}
                    step={0.01}
                    min={min as number}
                    max={max as number}
                    aria-label="Default"
                    valueLabelDisplay="auto"
                    onChange={(e) => {
                        handleJointValueChange(
                            (e.target as HTMLInputElement).value,
                        );
                    }}
                    onBlur={threeScene.forceUpdateCode}
                />
            </>
        );
    };

    return (
        <Section title="Joint">
            <Property>{`Parent Link: ${selectedObject.parentName}`}</Property>
            {!selectedObject.isRootFrame && (
                <Property name="Joint Type">
                    <Parameter
                        title=""
                        type="select"
                        value={selectedObject.jointType}
                        onSelectChange={handleJointTypeChange}
                        className={styles.select}
                        options={[
                            { value: "fixed", option: "Fixed" },
                            { value: "revolute", option: "Revolute" },
                            { value: "continuous", option: "Continuous" },
                            { value: "prismatic", option: "Prismatic" },
                        ]}
                    />
                </Property>
            )}
            <Property>
                <button
                    className={styles.button}
                    onClick={handleChangeAxisAngle}
                    onBlur={reattachLink}
                    type="button"
                >
                    Change Axis Angle
                </button>
                <button
                    className={styles.button}
                    onClick={handleChangeAxisOrigin}
                    onBlur={reattachLink}
                    type="button"
                >
                    Change Axis Origin
                </button>
            </Property>
            <OffsetParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
            {determineJointFromType(selectedObject.jointType)}
        </Section>
    );
}
