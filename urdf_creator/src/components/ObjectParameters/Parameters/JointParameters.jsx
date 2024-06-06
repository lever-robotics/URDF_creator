import React, { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";
import MuiInput from "@mui/material/Input";

export default function JointParameters({ selectedObject, setJoint, stateFunctions }) {
    const [jointType, setJointType] = useState(selectedObject.joint.type);

    const handleJointTypeChange = (e) => {
        setJointType(e.target.value);
        setJoint(selectedObject, e.target.value);
    };

    useEffect(() => {
        setJointType(selectedObject.joint.type);
    }, [selectedObject]);

    return (
        <div>
            <strong>Joint Information:</strong>
            <div>
                <strong>Parent Link:</strong>
                <span> {selectedObject.parent.parent.userData.name}</span>
            </div>
            <div>
                <strong>Joint Type:</strong>
                <select value={jointType} onChange={handleJointTypeChange}>
                    <option value="fixed">Fixed</option>
                    <option value="revolute">Revolute</option>
                    <option value="continuous">Continuous</option>
                    <option value="prismatic">Prismatic</option>
                    {/* <option value="planar">Planar</option>
                    <option value="floating">Floating</option> */}
                </select>
            </div>
            {jointType === "revolute" && <RevoluteOptions selectedObject={selectedObject} stateFunctions={stateFunctions} />}
            {jointType === "continuous" && <ContinuousOptions selectedObject={selectedObject} stateFunctions={stateFunctions} />}
            {jointType === "prismatic" && <PrismaticOptions selectedObject={selectedObject} stateFunctions={stateFunctions} />}
        </div>
    );
}

const RevoluteOptions = ({ selectedObject, stateFunctions }) => {
    // min value
    const [min, setMin] = useState(-6.28);
    // max value
    const [max, setMax] = useState(6.28);
    // current value
    const [current, setCurrent] = useState(0);

    const setRotationAboutJointAxis = stateFunctions.setRotationAboutJointAxis
    const startRotateJoint = stateFunctions.startRotateJoint;
    const startMoveJoint = stateFunctions.startMoveJoint;

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setRotationAboutJointAxis(selectedObject, value)
    };

    return (
        <>
            <button
                onClick={() => {
                    startRotateJoint(selectedObject);
                }}
            >
                Change Axis Angle
            </button>
            <button
                onClick={() => {
                    startMoveJoint(selectedObject);
                }}
            >
                Change Axis Origin
            </button>
            <div>
                <span>
                    min:
                    <MuiInput
                        value={min}
                        size="small"
                        onChange={(e) => setMin(parseFloat(e.target.value))}
                        inputProps={{
                            step: 0.1,
                            min: -6.28,
                            max: max,
                            type: "number",
                            "aria-labelledby": "input-slider",
                        }}
                    />
                </span>
                <span>
                    max:
                    <MuiInput
                        value={max}
                        size="small"
                        onChange={(e) => setMax(parseFloat(e.target.value))}
                        inputProps={{
                            step: 0.1,
                            min: min,
                            max: 6.28,
                            type: "number",
                            "aria-labelledby": "input-slider",
                        }}
                    />
                </span>
            </div>

            <Slider step={0.01} value={current} min={min} max={max} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};

const ContinuousOptions = ({ selectedObject, stateFunctions }) => {
    // current value
    const [current, setCurrent] = useState(0);

    const setRotationAboutJointAxis = stateFunctions.setRotationAboutJointAxis
    const startRotateJoint = stateFunctions.startRotateJoint;
    const startMoveJoint = stateFunctions.startMoveJoint;

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setRotationAboutJointAxis(selectedObject, value)
    };


    return (
        <>
            <button
                onClick={() => {
                    startRotateJoint(selectedObject);
                }}
            >
                Change Axis Angle
            </button>
            <button
                onClick={() => {
                    startMoveJoint(selectedObject);
                }}
            >
                Change Axis Origin
            </button>
            <div>
            </div>

            <Slider step={0.01} value={current} min={-3.14} max={3.14} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};

const PrismaticOptions = ({ selectedObject, stateFunctions }) => {
    // min value
    const [min, setMin] = useState(-1);
    // max value
    const [max, setMax] = useState(1);
    // current value
    const [current, setCurrent] = useState(0);

    const setPositionAcrossJointAxis = stateFunctions.setPositionAcrossJointAxis
    const startRotateJoint = stateFunctions.startRotateJoint;
    const startMoveJoint = stateFunctions.startMoveJoint;

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setPositionAcrossJointAxis(selectedObject, value)
    };

    return (
        <>
            <button
                onClick={() => {
                    startRotateJoint(selectedObject);
                }}
            >
                Change Axis Angle
            </button>
            <button
                onClick={() => {
                    startMoveJoint(selectedObject);
                }}
            >
                Change Axis Origin
            </button>
            <div>
                <span>
                    min:
                    <MuiInput
                        value={min}
                        size="small"
                        onChange={(e) => setMin(parseFloat(e.target.value))}
                        inputProps={{
                            step: 0.1,
                            min: -6.28,
                            max: max,
                            type: "number",
                            "aria-labelledby": "input-slider",
                        }}
                    />
                </span>
                <span>
                    max:
                    <MuiInput
                        value={max}
                        size="small"
                        onChange={(e) => setMax(parseFloat(e.target.value))}
                        inputProps={{
                            step: 0.1,
                            min: min,
                            max: 6.28,
                            type: "number",
                            "aria-labelledby": "input-slider",
                        }}
                    />
                </span>
            </div>

            <Slider step={0.01} value={current} min={min} max={max} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};