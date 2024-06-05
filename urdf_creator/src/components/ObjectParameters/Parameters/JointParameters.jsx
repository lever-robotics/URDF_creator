import React, { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";
import MuiInput from "@mui/material/Input";

export default function JointParameters({ selectedObject, setJoint, startMoveJoint, startRotateJoint }) {
    const [jointType, setJointType] = useState(selectedObject.jointAxis.type);

    const handleJointTypeChange = (e) => {
        setJointType(e.target.value);
        setJoint(selectedObject, e.target.value);
    };

    useEffect(() => {
        setJointType(selectedObject.jointAxis.type);
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
            {jointType === "revolute" && <RevoluteOptions startMoveJoint={startMoveJoint} startRotateJoint={startRotateJoint} selectedObject={selectedObject} />}
        </div>
    );
}

const RevoluteOptions = ({ startMoveJoint, startRotateJoint, selectedObject }) => {
    // min value
    const [min, setMin] = useState(-90);
    // max value
    const [max, setMax] = useState(90);
    // current value
    const [current, setCurrent] = useState(0);

    const handleSlider = (e) => {
        setCurrent(parseInt(e.target.value));
    };

    return (
        <>
            <button
                onClick={() => {
                    console.log("selected");
                    console.log(selectedObject);
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
                        onChange={(e) => setMin(e.target.value)}
                        inputProps={{
                            step: 10,
                            min: -360,
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
                        onChange={(e) => setMax(e.target.value)}
                        inputProps={{
                            step: 10,
                            min: "{min}",
                            max: 360,
                            type: "number",
                            "aria-labelledby": "input-slider",
                        }}
                    />
                </span>
            </div>

            <Slider value={current} min={min} max={max} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};
