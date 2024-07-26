import React, { useState, useEffect } from "react";
import Inertia from "../../../Models/Inertia";
import Parameter from "./Parameter";
import ToggleSection from "../ToggleSection";
import { handleDownload } from "../../../utils/HandleDownload";

function InertiaParameters({ selectedObject, stateFunctions }) {
    const { mass, ixx, ixy, ixz, iyy, iyz, izz } = selectedObject.inertia;

    const handleInertiaChange = (e) => {
        const inertia = parseFloat(e.target.value);
        const type = e.target.title.toLowerCase().replace(":", "");

        if (isNaN(inertia)) return;

        stateFunctions.setInertia(selectedObject, type, inertia);
    };

    const handleMassChange = (e) => {
        if (isNaN(e.target.value)) return;
        stateFunctions.setMass(selectedObject, parseFloat(e.target.value));
    };

    return (
        <ToggleSection title="Inertia Parameters">
            <ul>
                <Parameter
                    title="Mass:"
                    type="number"
                    value={mass}
                    onChange={handleMassChange}
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
                    value={ixx}
                    onChange={handleInertiaChange}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
                <Parameter
                    title="Ixy:"
                    type="number"
                    value={ixy}
                    onChange={handleInertiaChange}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
                <Parameter
                    title="Ixz:"
                    type="number"
                    value={ixz}
                    onChange={handleInertiaChange}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
                <Parameter
                    title="Iyy:"
                    type="number"
                    value={iyy}
                    onChange={handleInertiaChange}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
                <Parameter
                    title="Iyz:"
                    type="number"
                    value={iyz}
                    onChange={handleInertiaChange}
                    units={
                        <>
                            km<span className="large-dot">·</span>m<sup>2</sup>
                        </>
                    }
                />
                <Parameter
                    title="Izz:"
                    type="number"
                    value={izz}
                    onChange={handleInertiaChange}
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
