import ToggleSection from "../ToggleSection";
import Parameter from "./Parameter";

function ScaleParameters({ selectedObject, stateFunctions }) {

    const handleScaleChange = (e) => {
        const axis = e.target.title.toLowerCase().replace(":", "");
        let newValue = parseFloat(e.target.value);

        if(axis === 'radius'){
            newValue = newValue * 2;
        }

        if(newValue <= 0){
            newValue = .001;
        }

        if (isNaN(newValue)) return;
        console.log(newValue);

        stateFunctions.transformObject(
            selectedObject,
            "scale",
            axis,
            newValue
        );
    };

    const props = {
        type: "number",
        unit: "m",
        onChange: handleScaleChange,
    };

    const determineParametersFromShape = (shape) => {
        switch (shape) {
            case "sphere":
                return (
                    <Parameter
                        title="Radius:"
                        value={selectedObject.scale.x / 2}
                        {...props}
                    />
                );
            case "cylinder":
                return (
                    <>
                        <Parameter
                            title="Radius:"
                            value={selectedObject.scale.x / 2}
                            {...props}
                        />
                        <Parameter
                            title="Height:"
                            value={selectedObject.scale.z}
                            {...props}
                        />
                    </>
                );
            case "cube":
                return (
                    <>
                        <Parameter
                            title="X:"
                            value={selectedObject.scale.x}
                            {...props}
                        />
                        <Parameter
                            title="Y:"
                            value={selectedObject.scale.y}
                            {...props}
                        />
                        <Parameter
                            title="Z:"
                            value={selectedObject.scale.z}
                            {...props}
                        />
                    </>
                );
            default:
                throw Error("Shape not supported");
        }
    };

    return (
        <ToggleSection title="Scale">
            <ul>{determineParametersFromShape(selectedObject.shape)}</ul>
        </ToggleSection>
    );
}

export default ScaleParameters;
