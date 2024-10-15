import Frame from "../../../../Models/Frame";
import type ParameterProps from "../ParameterProps";
import Parameter from "./Parameter";
import Property from "./Property";
import Section from "./Section";

const OffsetParameters: React.FC<ParameterProps> = ({
    selectedObject,
    threeScene,
}) => {
    if (!selectedObject) return;
    if (!(selectedObject instanceof Frame)) return;

    const x = selectedObject.offset.x;
    const y = selectedObject.offset.y;
    const z = selectedObject.offset.z;

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

    return (
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
    );
};

export default OffsetParameters;
