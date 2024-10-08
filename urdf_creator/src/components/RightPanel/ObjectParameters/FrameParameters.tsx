import React, { useState, useEffect } from "react";
import { deregisterName, registerName } from "../../ThreeDisplay/TreeUtils";
import styles from "./ObjectParameters.module.css";
import ThreeScene from "../../ThreeDisplay/ThreeScene";
import Frame from "../../../Models/Frame";
import JointParameters from "./JointParameters";
import PositionParameters from "./Parameters/PositionParameters";
import RotationParameters from "./Parameters/RotationParameters";
import Section from "./Parameters/Section";
import Property from "./Parameters/Property";
import Parameter from "./Parameters/Parameter";

export default function FrameParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const [error, setError] = useState("");
    const [tempName, setTempName] = useState(selectedObject.name);

    //implement use effect to update when selected object changes
    useEffect(() => {
        setTempName(selectedObject.name);
        setError("");
    }, [JSON.stringify(selectedObject.name)]);

    const handleNameChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const newName = e.target.value;
        if (newName.includes(" ")) {
            setError("Name must have no spaces");
        } else {
            setTempName(newName);
        }
    };

    const handleNameBlur = (
        e:
            | React.FocusEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>
    ) => {
        const newName = e.currentTarget.value;
        if (newName === selectedObject.name) {
            setError("");
        } else if (threeScene.objectNames.includes(newName)) {
            setError("Name must be unique");
        } else if (newName === "") {
            setError("Name cannot be empty");
        } else {
            deregisterName(selectedObject.name, threeScene.objectNames);
            selectedObject.name = registerName(newName, threeScene.objectNames);
            threeScene.forceUpdateBoth();
            setError("");
        }
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            handleNameBlur(e);
        }
    };

    const props = {
        title: "",
        type: "text",
        value: tempName,
        onChange: handleNameChange,
        onBlur: handleNameBlur,
        onKeyDown: handleKeyDown,
        readOnly: selectedObject.name === "base_link",
        className: selectedObject.isRootFrame ? styles.rootName : styles.name,
    };

    // <Parameter
    //             title="X:"
    //             type="text"
    //             units="m"
    //             value={tempX}
    //             onChange={handlePositionChange}
    //             onBlur={handlePositionBlur}
    //             onKeyDown={handleKeyDown}
    //         />

    return (
        <Section>
            <Property name={"Name"}>
                <Parameter {...props} />
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
