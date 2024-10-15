import type React from "react";
import { useEffect, useState } from "react";
import type Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import { deregisterName, registerName } from "../../ThreeDisplay/TreeUtils";
import JointParameters from "./JointParameters";
import styles from "./ObjectParameters.module.css";
import Parameter from "./Parameters/Parameter";
import PositionParameters from "./Parameters/PositionParameters";
import Property from "./Parameters/Property";
import RotationParameters from "./Parameters/RotationParameters";
import Section from "./Parameters/Section";

export default function FrameParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const [error, setError] = useState("");

    const validateInput = (input: string) => {
        if (input.includes(" ")) {
            setError("Name must have no spaces");
        } else {
            setError("");
        }
        return input;
    };

    const handleBlur = (parameter: string, value: string) => {
        if (value === selectedObject.name) {
            setError("");
        } else if (threeScene.objectNames.includes(value)) {
            setError("Name must be unique");
        } else if (value === "") {
            setError("Name cannot be empty");
        } else {
            deregisterName(selectedObject.name, threeScene.objectNames);
            selectedObject.name = registerName(value, threeScene.objectNames);
            threeScene.forceUpdateBoth();
            setError("");
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Section>
            <Property name={"Name"}>
                <Parameter
                    title=""
                    kind="text"
                    parameter="name"
                    value={selectedObject.name}
                    handleBlur={handleBlur}
                    validateInput={validateInput}
                    readOnly={selectedObject.name === "base_link"}
                    className={
                        selectedObject.isRootFrame
                            ? styles.rootName
                            : styles.name
                    }
                />
            </Property>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
            <PositionParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
            <RotationParameters
                selectedObject={selectedObject}
                threeScene={threeScene}
            />
        </Section>
    );
}

export const WorldFrameParameters = ({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) => {
    if (!selectedObject) return;
    const [error, setError] = useState("");

    const validateInput = (input: string) => {
        if (input.includes(" ")) {
            setError("Name must have no spaces");
        } else {
            setError("");
        }
        return input;
    };

    const handleBlur = (parameter: string, value: string) => {
        if (value === selectedObject.name) {
            setError("");
        } else if (threeScene.objectNames.includes(value)) {
            setError("Name must be unique");
        } else if (value === "") {
            setError("Name cannot be empty");
        } else {
            deregisterName(selectedObject.name, threeScene.objectNames);
            selectedObject.name = registerName(value, threeScene.objectNames);
            threeScene.forceUpdateBoth();
            setError("");
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Section>
            <Property name={"Name"}>
                <Parameter
                    title=""
                    kind="text"
                    parameter="name"
                    value={selectedObject.name}
                    handleBlur={handleBlur}
                    validateInput={validateInput}
                    readOnly={selectedObject.name === "base_link"}
                    className={
                        selectedObject.isRootFrame
                            ? styles.rootName
                            : styles.name
                    }
                />
            </Property>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
        </Section>
    );
};
