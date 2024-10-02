import React from "react";
import "./parameters_style.css";
import "../ObjectParameters.css";
import Parameter from "./Parameter";
import Section from "../Section";
import { deregisterName, registerName } from "../../../ThreeDisplay/TreeUtils";
import ItemParameterProps from "../ItemParameterProps";
import { Collision, Visual } from "../../../../Models/VisualCollision";

export default function MaterialParameters({ threeScene, selectedItem, selectedObject }: ItemParameterProps) {
    if (!selectedObject) return;

    const handleColorChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        (selectedItem! as Visual | Collision).setColorByHex(e.target.value);
    };

    const handleColorBlur = () => {
        threeScene.forceUpdateScene();
    };

    return (
        <Section title="Material Parameters">
            <ul>
                <Parameter
                    title={"Color:"}
                    type="color"
                    value={"#"+ (selectedItem! as Visual | Collision).color.getHexString()}
                    onChange={handleColorChange}
                    onBlur={handleColorBlur}
                />
            </ul>
        </Section>
    );
}
