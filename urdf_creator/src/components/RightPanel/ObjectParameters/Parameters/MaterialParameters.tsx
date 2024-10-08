import React from "react";
import Parameter from "./Parameter";
import Section from "./Section";
import { Collision, Visual } from "../../../../Models/VisualCollision";
import ThreeScene from "../../../ThreeDisplay/ThreeScene";
import Property from "./Property";

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
        threeScene.forceUpdateScene();
    };

    return (
        <Property name="Material">
            <Parameter
                title={"Color:"}
                type="color"
                value={
                    "#" +
                    (selectedObject as Visual | Collision).color.getHexString()
                }
                onChange={handleColorChange}
                onBlur={handleColorBlur}
            />
        </Property>
    );
}
