import type React from "react";
import { useEffect, useState } from "react";
import type Frame from "../../../Models/Frame";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";
import styles from "./ObjectParameters.module.css";
import ParameterProps from "./ParameterProps";
import Parameter from "./Parameters/Parameter";
import Property from "./Parameters/Property";
import Section from "./Parameters/Section";

function InertiaParameters({
    threeScene,
    selectedObject,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject.inertia) return;
    const inertia = selectedObject.inertia;

    const handleBlur = (parameter: string, value: number) => {
        if (parameter === "mass") {
            selectedObject.mass = value;
        } else {
            selectedObject.inertia.setCustomInertia(parameter, value);
        }
        threeScene.forceUpdateCode();
    };

    return (
        <Section title="Inertia">
            <Property name="Properties">
                <Parameter
                    title="Mass:"
                    kind="number"
                    parameter="mass"
                    value={inertia.mass}
                    handleBlur={handleBlur}
                    units="kg"
                />
                <div className={styles.momentOfInertia}>
                    <Parameter
                        title="Ixx:"
                        kind="number"
                        parameter="ixx"
                        value={inertia.ixx}
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Ixy:"
                        kind="number"
                        parameter="ixy"
                        value={inertia.ixy}
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Ixz:"
                        kind="number"
                        parameter="ixz"
                        value={inertia.ixz}
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Iyy:"
                        kind="number"
                        parameter="iyy"
                        value={inertia.iyy}
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Iyz:"
                        kind="number"
                        parameter="iyz"
                        value={inertia.iyz}
                        handleBlur={handleBlur}
                    />
                    <Parameter
                        title="Izz:"
                        kind="number"
                        parameter="izz"
                        value={inertia.izz}
                        handleBlur={handleBlur}
                    />
                </div>
            </Property>
        </Section>
    );
}

export default InertiaParameters;
