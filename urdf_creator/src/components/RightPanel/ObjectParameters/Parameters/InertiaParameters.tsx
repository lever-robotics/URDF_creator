import React, { useState, useEffect } from "react";
import Inertia from "../../../../Models/Inertia";
import Parameter from "./Parameter";
import Section from "../Section";
import "../ObjectParameters.css"
import { handleDownload } from "../../../../utils/HandleDownload";
import ParameterProps from "../ParameterProps";

function InertiaParameters({ threeScene, selectedObject }: ParameterProps) {
    if (!selectedObject) return;
    if (!selectedObject.inertia) return;
    const [tempMass, setTempMass] = useState<string>(selectedObject.inertia.mass.toString());
    const [tempIxx, setTempIxx] = useState(selectedObject.inertia.ixx);
    const [tempIxy, setTempIxy] = useState(selectedObject.inertia.ixy);
    const [tempIxz, setTempIxz] = useState(selectedObject.inertia.ixz);
    const [tempIyy, setTempIyy] = useState(selectedObject.inertia.iyy);
    const [tempIyz, setTempIyz] = useState(selectedObject.inertia.iyz);
    const [tempIzz, setTempIzz] = useState(selectedObject.inertia.izz);

    useEffect(() => {
        if (!selectedObject.inertia) return;
        setTempMass(selectedObject.inertia.mass.toString());
        setTempIxx(selectedObject.inertia.ixx);
        setTempIxy(selectedObject.inertia.ixy);
        setTempIxz(selectedObject.inertia.ixz);
        setTempIyy(selectedObject.inertia.iyy);
        setTempIyz(selectedObject.inertia.iyz);
        setTempIzz(selectedObject.inertia.izz);
    }, [JSON.stringify(selectedObject.inertia)]);

    const checkNegativeZero = (value: string) => {

        if(value === "-0"){
            return "0";
        }else{
            return value;
        }
    }

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

    const handleInertiaBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        const inertia = parseFloat(checkNegativeZero(e.currentTarget.value));
        const type = e.currentTarget.title.toLowerCase().replace(":", "");

        if (isNaN(inertia)) return;

        selectedObject.inertia?.setCustomInertia(type, inertia);
        threeScene.forceUpdateBoth();
    };

    const handleMassChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        setTempMass(e.target.value);
    };

    const handleMassBlur = (e: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
        // convert value to number then check if its a number
        if (isNaN(Number(e.currentTarget.value))) return;
        selectedObject.mass = parseFloat(checkNegativeZero(e.currentTarget.value));
        threeScene.forceUpdateBoth();
    };

    const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
        if (e.key === "Enter") {
            if (e.currentTarget.title.toLowerCase().replace(":", "") === "mass") {
                handleMassBlur(e);
            } else {
                handleInertiaBlur(e);
            }
        }
    };

    return (
        <Section title="Inertia Parameters">
            <ul>
                <Parameter
                    title="Mass:"
                    type="text"
                    value={tempMass}
                    onChange={handleMassChange}
                    onBlur={handleMassBlur}
                    onKeyDown={handleKeyDown}
                    units="kg"
                />
            </ul>
            <br />
            <br />
            <strong>Moment of Inertia:</strong>
            <ul>
                <Parameter
                    title="Ixx:"
                    type="text"
                    value={tempIxx}
                    onChange={handleInertiaChange}
                    onBlur={handleInertiaBlur}
                    onKeyDown={handleKeyDown}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
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
                            km<span className="large-dot">·</span>m<sup>2</sup>
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
                            km<span className="large-dot">·</span>m<sup>2</sup>
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
                            km<span className="large-dot">·</span>m<sup>2</sup>
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
                            km<span className="large-dot">·</span>m<sup>2</sup>
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
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
            </ul>
        </Section>
    );
}

export default InertiaParameters;
