import type React from "react";
import type { Collision, Visual } from "../../../../Models/VisualCollision";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

export default function MaterialParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Visual | Collision;
}) {
    const handleColorChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        (selectedObject as Visual | Collision).setColorByHex(e.target.value);
    };

    const handleColorBlur = () => {
        threeScene.forceUpdateCode();
    };

    return (
        <Property name="Material">
            <Parameter
                title={"Color:"}
                kind="color"
                parameter="color"
                value={`#${(selectedObject as Visual | Collision).color.getHexString()}`}
                handleChange={handleColorChange}
                handleBlur={handleColorBlur}
            />
        </Property>
    );
}
