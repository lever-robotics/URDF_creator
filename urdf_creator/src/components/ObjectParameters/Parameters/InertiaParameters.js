import React, { useState, useEffect } from "react";
import Inertia from "../../../Models/Inertia";
import Parameter from "./Parameter";
import ToggleSection from "../ToggleSection";
import { handleDownload } from "../../../utils/HandleDownload";

function InertiaParameters({ selectedObject, stateFunctions }) {
    const [tempMass, setTempMass] = useState(selectedObject.inertia.mass);
    const [tempIxx, setTempIxx] = useState(selectedObject.inertia.ixx);
    const [tempIxy, setTempIxy] = useState(selectedObject.inertia.ixy);
    const [tempIxz, setTempIxz] = useState(selectedObject.inertia.ixz);
    const [tempIyy, setTempIyy] = useState(selectedObject.inertia.iyy);
    const [tempIyz, setTempIyz] = useState(selectedObject.inertia.iyz);
    const [tempIzz, setTempIzz] = useState(selectedObject.inertia.izz);

    useEffect(() => {
        setTempMass(selectedObject.inertia.mass);
        setTempIxx(selectedObject.inertia.ixx);
        setTempIxy(selectedObject.inertia.ixy);
        setTempIxz(selectedObject.inertia.ixz);
        setTempIyy(selectedObject.inertia.iyy);
        setTempIyz(selectedObject.inertia.iyz);
        setTempIzz(selectedObject.inertia.izz);
    }, [JSON.stringify(selectedObject.inertia)]);

    const handleInertiaChange = (e) => {
        const type = e.target.title.toLowerCase().replace(":", "");
        const tempValue = e.target.value;

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

    const handleInertiaBlur = (e) => {
        const inertia = parseFloat(e.target.value);
        const type = e.target.title.toLowerCase().replace(":", "");

        if (isNaN(inertia)) return;

        stateFunctions.setInertia(selectedObject, type, inertia);
    };

    const handleMassChange = (e) => {
        setTempMass(e.target.value);
    };

    const handleMassBlur = (e) => {
        if (isNaN(e.target.value)) return;
        stateFunctions.setMass(selectedObject, parseFloat(e.target.value));
    };

    const handleKeyDown = (e) => {
        if (e.key === "Enter") {
            if (e.target.title.toLowerCase().replace(":", "") === "mass") {
                handleMassBlur(e);
            } else {
                handleInertiaBlur(e);
            }
        }
    };

    return (
        <ToggleSection title="Inertia Parameters">
            <ul>
                <Parameter
                    title="Mass:"
                    type="number"
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
                    type="number"
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
                    type="number"
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
                    type="number"
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
                    type="number"
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
                    type="number"
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
                    type="number"
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
        </ToggleSection>
    );
}

export default InertiaParameters;
