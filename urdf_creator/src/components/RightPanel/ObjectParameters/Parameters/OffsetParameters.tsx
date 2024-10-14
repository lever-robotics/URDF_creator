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

    // const [tempX, setTempX] = useState<ParameterValue>(selectedObject.offset.x);
    // const [tempY, setTempY] = useState<ParameterValue>(selectedObject.offset.y);
    // const [tempZ, setTempZ] = useState<ParameterValue>(selectedObject.offset.z);

    // //implement use effect to update when selected object changes
    // useEffect(() => {
    //     setTempX(
    //         Math.abs(selectedObject.offset.x) < 0.00001
    //             ? 0.0
    //             : Number.parseFloat(selectedObject.offset.x.toFixed(4)),
    //     );
    //     setTempY(
    //         Math.abs(selectedObject.offset.y) < 0.00001
    //             ? 0.0
    //             : Number.parseFloat(selectedObject.offset.y.toFixed(4)),
    //     );
    //     setTempZ(
    //         Math.abs(selectedObject.offset.z) < 0.00001
    //             ? 0.0
    //             : Number.parseFloat(selectedObject.offset.z.toFixed(4)),
    //     );
    // }, [
    //     selectedObject.offset.x,
    //     selectedObject.offset.y,
    //     selectedObject.offset.z,
    // ]);

    // const validateInput = (value: string) => {
    //     // If you click enter or away with invalid input then reset
    //     const newValue = Number.parseFloat(value);
    //     if (Number.isNaN(newValue)) {
    //         setTempX(selectedObject.position.x);
    //         setTempY(selectedObject.position.y);
    //         setTempZ(selectedObject.position.z);
    //         return false;
    //     }

    //     if (Object.is(newValue, -0)) {
    //         return 0;
    //     }
    //     return newValue;
    // };

    // const handleOffsetChange = (
    //     e:
    //         | React.ChangeEvent<HTMLInputElement>
    //         | React.KeyboardEvent<HTMLInputElement>,
    // ) => {
    //     const axis = e.currentTarget.title.toLowerCase().replace(":", "");
    //     const tempValue = e.currentTarget.value;
    //     switch (axis) {
    //         case "x":
    //             setTempX(tempValue);
    //             break;
    //         case "y":
    //             setTempY(tempValue);
    //             break;
    //         case "z":
    //             setTempZ(tempValue);
    //             break;
    //         default:
    //             break;
    //     }
    // };

    // const handleOffsetBlur = (
    //     e:
    //         | React.FocusEvent<HTMLInputElement>
    //         | React.KeyboardEvent<HTMLInputElement>,
    // ) => {
    //     const axis = e.currentTarget.title.toLowerCase().replace(":", "");
    //     const validValue = validateInput(e.currentTarget.value);
    //     if (validValue === false) return;
    //     const newOffset = selectedObject.offset.toArray();
    //     switch (axis) {
    //         case "x":
    //             newOffset[0] = validValue;
    //             setTempX(selectedObject.offset.x);
    //             break;
    //         case "y":
    //             newOffset[1] = validValue;
    //             setTempY(selectedObject.offset.y);
    //             break;
    //         case "z":
    //             newOffset[2] = validValue;
    //             setTempZ(selectedObject.offset.z);
    //             break;
    //     }
    //     selectedObject.offset.set(...newOffset);
    //     handleOffsetChange(e);
    // };

    // const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    //     if (e.key === "Enter") {
    //         handleOffsetBlur(e);
    //         (e.target as HTMLInputElement).blur();
    //     }
    // };

    // return (
    //     <Property name="Offset">
    //         <Parameter
    //             title="X:"
    //             type="text"
    //             units="m"
    //             value={tempX}
    //             onChange={handleOffsetChange}
    //             onBlur={handleOffsetBlur}
    //             onKeyDown={handleKeyDown}
    //         />
    //         <Parameter
    //             title="Y:"
    //             type="text"
    //             units="m"
    //             value={tempY}
    //             onChange={handleOffsetChange}
    //             onBlur={handleOffsetBlur}
    //             onKeyDown={handleKeyDown}
    //         />
    //         <Parameter
    //             title="Z:"
    //             type="text"
    //             units="m"
    //             value={tempZ}
    //             onChange={handleOffsetChange}
    //             onBlur={handleOffsetBlur}
    //             onKeyDown={handleKeyDown}
    //         />
    //     </Property>
    // );
};

export default OffsetParameters;
