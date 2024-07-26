import ToggleSection from '../ToggleSection';
import Parameter from './Parameter';

function PositionParameters({ selectedObject, stateFunctions }) {

    const handlePositionChange = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        const newValue = parseFloat(e.target.value);
        if (isNaN(newValue)) return;

        stateFunctions.transformObject(selectedObject, "position", axis, newValue);
    };

    return (
        <ToggleSection title="Position">
            <ul>
                <Parameter
                    title="X:"
                    type="number"
                    units="m"
                    value={selectedObject.position.x}
                    onChange={handlePositionChange}
                />
                <Parameter
                    title="Y:"
                    type="number"
                    units="m"
                    value={selectedObject.position.y}
                    onChange={handlePositionChange}
                />
                <Parameter
                    title="Z:"
                    type="number"
                    units="m"
                    value={selectedObject.position.z}
                    onChange={handlePositionChange}
                />
            </ul>
        </ToggleSection>
    );
}

export default PositionParameters;
