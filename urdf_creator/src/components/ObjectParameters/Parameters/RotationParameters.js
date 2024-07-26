import ToggleSection from "../ToggleSection";
import Parameter from "./Parameter";

function RotationParameters({ selectedObject, stateFunctions }) {
    // const [rotationX, setRotationX] = useState('');
    // const [rotationY, setRotationY] = useState('');
    // const [rotationZ, setRotationZ] = useState('');

    // useEffect(() => {
    //     if (selectedObject) {
    //         setRotationX(radToDeg(selectedObject.rotation.x).toFixed(2));
    //         setRotationY(radToDeg(selectedObject.rotation.y).toFixed(2));
    //         setRotationZ(radToDeg(selectedObject.rotation.z).toFixed(2));
    //     }
    // }, [JSON.stringify(selectedObject.rotation)]);

    const radToDeg = (radians) => (radians * 180) / Math.PI;
    const degToRad = (degrees) => (degrees * Math.PI) / 180;

    const handleRotationChange = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(e.target.value);
        if (isNaN(newValue)) return;

        stateFunctions.transformObject(
            selectedObject,
            "rotation",
            axis,
            degToRad(newValue)
        );
    };

    // const handleBlur = (prop, axis, value) => {
    //     handleChange(prop, axis, value);
    // };

    // const handleKeyDown = (e, prop, axis, value) => {
    //     if (e.key === 'Enter') {
    //         handleBlur(prop, axis, value);
    //     }
    // };

    return (
        <ToggleSection title="Rotation">
            <ul>
                <Parameter
                    title="X:"
                    type="number"
                    units="°degrees"
                    value={radToDeg(selectedObject.rotation.x).toFixed(2)}
                    onChange={handleRotationChange}
                />
                <Parameter
                    title="Y:"
                    type="number"
                    units="°degrees"
                    value={radToDeg(selectedObject.rotation.y).toFixed(2)}
                    onChange={handleRotationChange}
                />
                <Parameter
                    title="Z:"
                    type="number"
                    units="°degrees"
                    value={radToDeg(selectedObject.rotation.z).toFixed(2)}
                    onChange={handleRotationChange}
                />
            </ul>
        </ToggleSection>
    );
}

export default RotationParameters;
