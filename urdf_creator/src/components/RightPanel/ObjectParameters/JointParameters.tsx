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
    const [jointValue, setJointValue] = useState(0);

    const handleJointTypeChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
        const value = e.currentTarget.value;
        selectedObject.jointType = value as JointType;
        threeScene.forceUpdateCode();
    };

    const handleJointValueChange = (valueString: string) => {
        const value = Number.parseFloat(valueString);
        setJointValue(value);
        switch (selectedObject.jointType) {
            case "prismatic":
                selectedObject.translateAlongJointAxis(value);
                break;
            default:
                selectedObject.rotateAroundJointAxis(value);
                break;
        }
    };

    const handleChangeJointAngle = () => {
        threeScene.startRotateJoint(selectedObject);
    };

    const handleChangeJointOrigin = () => {
        threeScene.startMoveJoint(selectedObject);
    };

    const reattachLink = () => {
        threeScene.reattachLink(selectedObject);
    };

    const handleBlur = (parameter: string, value: number) => {
        switch (parameter) {
            case "min":
                selectedObject.min = value;
                break;
            case "max":
                selectedObject.max = value;
                break;
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Section title="Joint">
            <Property>{`Parent Link: ${selectedObject.parentName}`}</Property>
            {!selectedObject.isRootFrame && (
                <Property name="Joint Type">
                    <Parameter
                        title=""
                        kind="select"
                        parameter="jointType"
                        value={selectedObject.jointType}
                        handleChange={handleJointTypeChange}
                        options={[
                            { value: "fixed", optionType: "Fixed" },
                            { value: "revolute", optionType: "Revolute" },
                            { value: "continuous", optionType: "Continuous" },
                            { value: "prismatic", optionType: "Prismatic" },
                        ]}
                    />
                </Property>
            )}
            <Property>
                <button
                    className={styles.button}
                    onClick={handleChangeJointAngle}
                    onBlur={reattachLink}
                    type="button"
                >
                    Change Joint Angle
                </button>
                <button
                    className={styles.button}
                    onClick={handleChangeJointOrigin}
                    onBlur={reattachLink}
                    type="button"
                >
                    Change Joint Origin
                </button>
            </Property>
            <OffsetParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
            {selectedObject.jointType !== "fixed" && (
                <>
                    <Property name="Joint Limits">
                        <Parameter
                            title="Min:"
                            kind="number"
                            parameter="min"
                            value={selectedObject.min}
                            handleBlur={handleBlur}
                        />
                        <Parameter
                            title="Max:"
                            kind="number"
                            parameter="max"
                            value={selectedObject.max}
                            handleBlur={handleBlur}
                        />
                    </Property>
                    <Slider
                        value={jointValue}
                        step={0.01}
                        min={-2}
                        max={2}
                        aria-label="Default"
                        valueLabelDisplay="auto"
                        onChange={(e) => {
                            handleJointValueChange(
                                (e.target as HTMLInputElement).value,
                            );
                        }}
                        onBlur={() => handleJointValueChange("0")}
                    />
                </>
            )}
        </Section>
    );
}
