import Slider from "@mui/material/Slider";
import Parameter from "./Parameter";

export default function JointParameters({ selectedObject, stateFunctions }) {
    const handleJointTypeChange = (e) => {
        stateFunctions.setJointType(selectedObject, e.target.value);
    };

    // const handleSliderValue = () => {
    //     switch (selectedObject.jointType) {
    //         case "prismatic":
    //             return selectedObject.distanceAlongJointAxis();
    //         default:
    //             return selectedObject.angleAroundJointAxis();
    //     }
    // };
    const sliderValue = selectedObject.jointType === "prismatic" ? selectedObject.distanceAlongJointAxis() : selectedObject.angleAroundJointAxis();

    const handleSliderChange = (e) => {
        const value = parseFloat(e.target.value);
        switch (selectedObject.jointType) {
            case "prismatic":
                stateFunctions.translateAlongJointAxis(selectedObject, value);
                break;
            default:
                stateFunctions.rotateAroundJointAxis(selectedObject, value);
                break;
        }
    };

    const resetSlider = () => {
        stateFunctions.resetFromDisplayChanges(selectedObject);
    };

    const saveSlider = () => {
        stateFunctions.saveForDisplayChanges(selectedObject);
    };

    const handleChangeAxisAngle = () => {
        stateFunctions.startRotateJoint(selectedObject);
    };

    const handleChangeAxisOrigin = () => {
        stateFunctions.startMoveJoint(selectedObject);
    };

    const handleSetMinMax = (e) => {
        const type = e.target.title.toLowerCase().replace(":", "");
        const value = parseFloat(e.target.value);
        stateFunctions.setJointMinMax(selectedObject, type, value);
    };

    const reattachLink = () => {
        stateFunctions.reattachLink(selectedObject);
    }

    return (
        <div>
            <strong>Joint Information:</strong>
            <div>
                <strong>Parent Link:</strong>
                <span> {selectedObject.parentName}</span>
            </div>
            <div>
                <strong>Joint Type:</strong>
                <select
                    value={selectedObject.jointType}
                    onChange={handleJointTypeChange}>
                    <option value="fixed">Fixed</option>
                    <option value="revolute">Revolute</option>
                    <option value="continuous">Continuous</option>
                    <option value="prismatic">Prismatic</option>
                    {/* <option value="planar">Planar</option>
                    <option value="floating">Floating</option> */}
                </select>
            </div>
            {selectedObject.jointType !== "fixed" && (
                <>
                    <button onClick={handleChangeAxisAngle} onBlur={reattachLink}>
                        Change Axis Angle
                    </button>
                    <button
                        onClick={handleChangeAxisOrigin}
                        onBlur={reattachLink}
                    >
                        Change Axis Origin
                    </button>
                    {selectedObject.jointType !== "continuous" && (
                        <ul>
                            <Parameter
                                title="Min:"
                                size="small"
                                value={selectedObject.min}
                                onChange={handleSetMinMax}
                            />
                            <Parameter
                                title="Max:"
                                size="small"
                                value={selectedObject.max}
                                onChange={handleSetMinMax}
                            />
                        </ul>
                    )}
                    <Slider
                        step={0.01}
                        value={sliderValue}
                        min={selectedObject.min}
                        max={selectedObject.max}
                        aria-label="Default"
                        valueLabelDisplay="auto"
                        onChange={handleSliderChange}
                        onMouseDown={saveSlider}
                        onMouseUp={resetSlider}
                    />
                </>
            )}
        </div>
    );
}
