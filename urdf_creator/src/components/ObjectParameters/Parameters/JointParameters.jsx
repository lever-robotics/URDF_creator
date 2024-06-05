import React, { useEffect, useState } from "react";
import Axis from "../../../Models/Axis";

export default function JointParameters({ selectedObject, setJoint, startMoveJoint, startRotateJoint }) {
    const [jointType, setJointType] = useState(selectedObject.jointAxis.type);

    const handleJointTypeChange = (e) => {
        setJointType(e.target.value);
        const axis = new Axis({ type: e.target.value });
        setJoint(selectedObject, axis);
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
        </div>
    );
}

const RevoluteOptions = ({ startMoveJoint, startRotateJoint, selectedObject }) => {
    return (
        <>
            <button
                onClick={() => {
                    console.log("clicked");
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
        </>
    );
};
