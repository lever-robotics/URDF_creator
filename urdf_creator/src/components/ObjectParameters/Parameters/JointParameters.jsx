import React, { useEffect, useState } from "react";
import Slider from "@mui/material/Slider";

export default function JointParameters({ selectedObject, setJoint, stateFunctions }) {
    const [jointType, setJointType] = useState(selectedObject.joint.type);

    const handleJointTypeChange = (e) => {
        setJointType(e.target.value);
        selectedObject.joint.type = e.target.value;
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

const MinJointInput = ({ min, setMin, selectedObject, stateFunctions }) => {

    const [value, setValue] = useState(min);

    const setJointLimits = stateFunctions.setJointLimits;

    const handleChange = () => {
        setJointLimits(selectedObject, parseFloat(value), null);
        setMin(parseFloat(value))
    }
    return <span>
        min:
        <input
            value={value}
            size="small"
            onChange={(e) => setValue(e.target.value)}
            onBlur={handleChange}
            onKeyDown={(e) => {
                if (e.key === 'Enter') {
                    handleChange();
                }
            }}
        />
    </span>
}

const MaxJointInput = ({ max, setMax, selectedObject, stateFunctions }) => {

    const [value, setValue] = useState(max);

    const setJointLimits = stateFunctions.setJointLimits;

    const handleChange = () => {
        setJointLimits(selectedObject, null, parseFloat(value));
        setMax(parseFloat(value));
    }

    return <span>
        max:
        <input
            value={value}
            size="small"
            onChange={(e) => setValue(e.target.value)}
            onBlur={handleChange}
            onKeyDown={(e) => {
                if (e.key === 'Enter') {
                    handleChange();
                }
            }}
        />
    </span>
}

const RevoluteOptions = ({ selectedObject, stateFunctions }) => {
    // min value
    const [min, setMin] = useState(selectedObject.joint.min);
    // max value
    const [max, setMax] = useState(selectedObject.joint.max);
    // current value
    const [current, setCurrent] = useState(0);

    const setRotationAboutJointAxis = stateFunctions.setRotationAboutJointAxis;

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setRotationAboutJointAxis(selectedObject, value)
    };

    return (
        <>
            <RotateAxisButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <MoveOriginButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <div>
                <MinJointInput min={min} setMin={setMin} max={max} selectedObject={selectedObject} stateFunctions={stateFunctions} />
                <MaxJointInput min={min} max={max} setMax={setMax} selectedObject={selectedObject} stateFunctions={stateFunctions} />
            </div>
            <Slider step={0.01} value={current} min={min} max={max} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};

const ContinuousOptions = ({ selectedObject, stateFunctions }) => {
    // current value
    const [current, setCurrent] = useState(0);

    const setRotationAboutJointAxis = stateFunctions.setRotationAboutJointAxis

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setRotationAboutJointAxis(selectedObject, value)
    };


    return (
        <>
            <RotateAxisButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <MoveOriginButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <Slider step={0.01} value={current} min={-3.14} max={3.14} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};

const PrismaticOptions = ({ selectedObject, stateFunctions }) => {
    // min value
    const [min, setMin] = useState(selectedObject.joint.min);
    // max value
    const [max, setMax] = useState(selectedObject.joint.max);
    // current value
    const [current, setCurrent] = useState(0);

    const setPositionAcrossJointAxis = stateFunctions.setPositionAcrossJointAxis

    const handleSlider = (e) => {
        const value = parseFloat(e.target.value);
        setCurrent(value);
        setPositionAcrossJointAxis(selectedObject, value)
    };

    return (
        <>
            <RotateAxisButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <MoveOriginButton selectedObject={selectedObject} stateFunctions={stateFunctions} />
            <div>
                <MinJointInput min={min} setMin={setMin} max={max} selectedObject={selectedObject} stateFunctions={stateFunctions} />
                <MaxJointInput min={min} max={max} setMax={setMax} selectedObject={selectedObject} stateFunctions={stateFunctions} />
            </div>

            <Slider step={0.01} value={current} min={min} max={max} aria-label="Default" valueLabelDisplay="auto" onChange={handleSlider} />
        </>
    );
};

const MoveOriginButton = ({ stateFunctions, selectedObject }) => {

    const startMoveJoint = stateFunctions.startMoveJoint;
    const unlockCurrentOffsetChangeNode = stateFunctions.unlockCurrentOffsetChangeNode;

    return <button
        onClick={() => {
            startMoveJoint(selectedObject);
        }}
        onBlur={unlockCurrentOffsetChangeNode}
    >
        Change Axis Origin
    </button>
}

const RotateAxisButton = ({ stateFunctions, selectedObject }) => {
    const startRotateJoint = stateFunctions.startRotateJoint;

    return <button
        onClick={() => {
            startRotateJoint(selectedObject);
        }}
    >
        Change Axis Angle
    </button>
}