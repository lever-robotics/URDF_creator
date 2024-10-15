import { useEffect, useState } from "react";
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
    const [offset, setOffset] = useState(0);

    useEffect(() => {
        const updateOffset = () => {
            setOffset((prev) => prev + 1);
        };

        const updateLinkDetached = () => {
            setLinkDetached(threeScene.linkDetached);
        };

        threeScene.addEventListener("parameters", updateOffset);
        threeScene.addEventListener("linkAttached", updateLinkDetached);

        return () => {
            threeScene.removeEventListener("parameters", updateOffset);
            threeScene.removeEventListener("linkAttached", updateLinkDetached);
        };
    }, [threeScene]);

    if (!selectedObject) return;
    if (!(selectedObject instanceof Frame)) return;

    const calculateOffset = () => {
        const originalJointPosition = selectedObject.offset.clone(); // Clone to avoid modifying the original position
        const newJointPosition = selectedObject.position;
        return originalJointPosition.sub(newJointPosition);
    };

    const x = linkDetached ? calculateOffset().x : selectedObject.offset.x;
    const y = linkDetached ? calculateOffset().y : selectedObject.offset.y;
    const z = linkDetached ? calculateOffset().z : selectedObject.offset.z;

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

    return (
        <>
            <Property>
                <button
                    className={styles.button}
                    onClick={handleChangeJointOrigin}
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
