import { useState } from "react";
import Frame from "../../../../Models/Frame";
import styles from "../ObjectParameters.module.css";
import type ParameterProps from "../ParameterProps";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

const OffsetParameters: React.FC<ParameterProps> = ({
    selectedObject,
    threeScene,
}) => {
    const [linkDetached, setLinkDetached] = useState(false);

    if (!selectedObject) return;
    if (!(selectedObject instanceof Frame)) return;

    const calculateOffset = () => {
        const originalJointPosition = selectedObject.tempOffset.clone(); // Clone to avoid modifying the original position
        const newJointPosition = selectedObject.position;
        return originalJointPosition.sub(newJointPosition);
    };

    const x = linkDetached ? calculateOffset().x : selectedObject.offset.x;
    const y = linkDetached ? calculateOffset().y : selectedObject.offset.y;
    const z = linkDetached ? calculateOffset().z : selectedObject.offset.z;
    console.log(calculateOffset());
    console.log(x, y, z);

    const handleBlur = (parameter: string, value: number) => {
        switch (parameter) {
            case "x": {
                selectedObject.offset.setX(value);
                break;
            }
            case "y": {
                selectedObject.offset.setY(value);
                break;
            }
            case "z": {
                selectedObject.offset.setZ(value);
                break;
            }
        }
        threeScene.forceUpdateCode();
    };

    const handleChangeJointOrigin = () => {
        threeScene.startMoveJoint(selectedObject);
        setLinkDetached(true);
    };

    const reattachLink = () => {
        threeScene.reattachLink(selectedObject);
        setLinkDetached(false);
    };

    return (
        <>
            <Property>
                <button
                    className={styles.button}
                    onClick={handleChangeJointOrigin}
                    onBlur={reattachLink}
                    type="button"
                >
                    Change Joint Origin
                </button>
            </Property>
            <Property name="Offset">
                <Parameter
                    title="X:"
                    kind="number"
                    parameter="x"
                    value={x}
                    handleBlur={handleBlur}
                    units="m"
                />
                <Parameter
                    title="Y:"
                    kind="number"
                    parameter="y"
                    value={y}
                    handleBlur={handleBlur}
                    units="m"
                />
                <Parameter
                    title="Z:"
                    kind="number"
                    parameter="z"
                    value={z}
                    handleBlur={handleBlur}
                    units="m"
                />
            </Property>
        </>
    );
};

export default OffsetParameters;
