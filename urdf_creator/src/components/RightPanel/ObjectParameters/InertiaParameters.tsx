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
    const [tempMass, setTempMass] = useState<string>(inertia.mass.toString());
    const [tempIxx, setTempIxx] = useState(inertia.ixx);
    const [tempIxy, setTempIxy] = useState(inertia.ixy);
    const [tempIxz, setTempIxz] = useState(inertia.ixz);
    const [tempIyy, setTempIyy] = useState(inertia.iyy);
    const [tempIyz, setTempIyz] = useState(inertia.iyz);
    const [tempIzz, setTempIzz] = useState(inertia.izz);

    useEffect(() => {
        setTempMass(inertia.mass.toString());
        setTempIxx(inertia.ixx);
        setTempIxy(inertia.ixy);
        setTempIxz(inertia.ixz);
        setTempIyy(inertia.iyy);
        setTempIyz(inertia.iyz);
        setTempIzz(inertia.izz);
    }, [
        inertia.mass,
        inertia.ixx,
        inertia.ixy,
        inertia.ixz,
        inertia.iyy,
        inertia.iyz,
        inertia.izz,
    ]);

    const checkNegativeZero = (value: string) => {
        if (value === "-0") {
            return "0";
        }
        return value;
    };

    const handleInertiaChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const type = e.currentTarget.title.toLowerCase().replace(":", "");
        const tempValue = Number(e.currentTarget.value);

        switch (type) {
            case "ixx":
                setTempIxx(tempValue);
                break;
            case "ixy":
                setTempIxy(tempValue);
                break;
            case "ixz":
                setTempIxz(tempValue);
                break;
            case "iyy":
                setTempIyy(tempValue);
                break;
            case "iyz":
                setTempIyz(tempValue);
                break;
            case "izz":
                setTempIzz(tempValue);
                break;
        }
    };

    const handleInertiaBlur = (
        e:
            | React.FocusEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>,
    ) => {
        const inertia = Number.parseFloat(
            checkNegativeZero(e.currentTarget.value),
        );
        const type = e.currentTarget.title.toLowerCase().replace(":", "");

        if (Number.isNaN(inertia)) return;

        selectedObject.inertia?.setCustomInertia(type, inertia);
        threeScene.forceUpdateBoth();
    };

    const handleMassChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        setTempMass(e.target.value);
    };

    const handleMassBlur = (
        e:
            | React.FocusEvent<HTMLInputElement>
            | React.KeyboardEvent<HTMLInputElement>,
    ) => {
        // convert value to number then check if its a number
        if (Number.isNaN(Number(e.currentTarget.value))) return;
        selectedObject.mass = Number.parseFloat(
            checkNegativeZero(e.currentTarget.value),
        );
        threeScene.forceUpdateBoth();
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            if (
                e.currentTarget.title.toLowerCase().replace(":", "") === "mass"
            ) {
                handleMassBlur(e);
            } else {
                handleInertiaBlur(e);
            }
        }
    };

    return (
        <Section title="Inertia">
            <Property name="Properties">
                <Parameter
                    title="Mass:"
                    type="text"
                    value={tempMass}
                    onChange={handleMassChange}
                    onBlur={handleMassBlur}
                    onKeyDown={handleKeyDown}
                    units="kg"
                />
                <div className={styles.momentOfInertia}>
                    <Parameter
                        title="Ixx:"
                        type="text"
                        value={tempIxx}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                    <Parameter
                        title="Ixy:"
                        type="text"
                        value={tempIxy}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                    <Parameter
                        title="Ixz:"
                        type="text"
                        value={tempIxz}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                    <Parameter
                        title="Iyy:"
                        type="text"
                        value={tempIyy}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                    <Parameter
                        title="Iyz:"
                        type="text"
                        value={tempIyz}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                    <Parameter
                        title="Izz:"
                        type="text"
                        value={tempIzz}
                        onChange={handleInertiaChange}
                        onBlur={handleInertiaBlur}
                        onKeyDown={handleKeyDown}
                        units={
                            <>
                                km<span className="large-dot">·</span>m
                                <sup>2</sup>
                            </>
                        }
                    />
                </div>
            </Property>
        </Section>
    );
}

export default InertiaParameters;
