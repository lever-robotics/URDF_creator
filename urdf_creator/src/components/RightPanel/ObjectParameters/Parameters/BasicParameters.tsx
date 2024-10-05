import React, { useState, useEffect } from "react";
import Parameter from "./Parameter";
import ParameterProps from "../ParameterProps";
import { deregisterName, registerName } from "../../../ThreeDisplay/TreeUtils";
import styles from "../ObjectParameters.module.css"
import ThreeScene from "../../../ThreeDisplay/ThreeScene";
import Frame from "../../../../Models/Frame";

export default function BasicParameters({ threeScene, selectedObject }: {threeScene: ThreeScene, selectedObject: Frame}) {
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

    const handleNameBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const newName = e.currentTarget.value;
        if(newName === selectedObject.name){
            setError("");
        }else if(threeScene.objectNames.includes(newName)){
            setError("Name must be unique");
        }else if (newName === "") {
            setError("Name cannot be empty");
        } else {
            deregisterName(selectedObject.name, threeScene.objectNames);
            selectedObject.name = registerName(newName, threeScene.objectNames);
            threeScene.forceUpdateBoth();
            setError("");
        }
    }

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if(e.key === "Enter"){
            handleNameBlur(e);
        }
    }

    const handleColorBlur = () => {
        threeScene.forceUpdateScene();
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
    }

    return (            
        <div className={styles.basic}>
            Name
            <div className={styles.parameter}>
                <strong className={styles.paramLabel}>{props.title}</strong>
                <input {...props} className={props.className}/>
            </div>
            {error && (
                <span style={{ color: "red", marginLeft: "5px" }}>{error}</span>
            )}{" "}
        </div>
    );
}
