import type React from "react";
import { useState } from "react";
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
    const [update, setUpdate] = useState(0);

    const handleColorChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        (selectedObject as Visual | Collision).setColorByHex(e.target.value);
    };

    const handleBlur = () => {
        setUpdate((prev) => prev + 1);
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
                handleBlur={handleBlur}
            />
        </Property>
    );
}
