import type React from "react";
import type Frame from "../../../../Models/Frame";
import { Camera } from "../../../../Models/SensorsClass";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import ParameterProps from "../ParameterProps";
import Parameter from "../Parameters/Parameter";
import Property from "../Parameters/Property";

function CameraParameters({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const camera = selectedObject.sensor;
    if (!(camera instanceof Camera)) return;

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        switch (name) {
            case "cameraName":
                camera.cameraName = value;
                break;
            case "imageTopicName":
                camera.imageTopicName = value;
                break;
            case "cameraInfoTopicName":
                camera.cameraInfoTopicName = value;
                break;
            case "format":
                camera.format = value;
                break;
        }
        const newValue = Number.parseFloat(value);
        switch (name) {
            case "horizontalFov":
                camera.horizontal_fov = newValue;
                break;
            case "width":
                camera.width = newValue;
                break;
            case "height":
                camera.height = newValue;
                break;
            case "near":
                camera.near = newValue;
                break;
            case "far":
                camera.far = newValue;
                break;
            case "gaussianNoise":
                camera.gaussianNoise = newValue;
                break;
            case "updateRate":
                camera.updateRate = newValue;
                break;
        }

        threeScene.forceUpdateCode();
    };

    return null;
    // return (
    //     <>
    //         <Property>
    //             <Parameter
    //                 title="Camera Name:"
    //                 type="text"
    //                 name="cameraName"
    //                 value={camera.cameraName}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Horizontal FOV:"
    //                 type="number"
    //                 name="horizontalFov"
    //                 value={camera.horizontal_fov}
    //                 onChange={handleChange}
    //                 units={"&deg"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Image Width:"
    //                 type="number"
    //                 name="width"
    //                 value={camera.width}
    //                 onChange={handleChange}
    //                 units={"px"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Image Height:"
    //                 type="number"
    //                 name="height"
    //                 value={camera.height}
    //                 onChange={handleChange}
    //                 units={"px"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Image Format:"
    //                 type="text"
    //                 name="format"
    //                 value={camera.format}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Clip Near:"
    //                 type="number"
    //                 name="near"
    //                 value={camera.near}
    //                 onChange={handleChange}
    //                 units={"m"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Clip Far:"
    //                 type="number"
    //                 name="far"
    //                 value={camera.far}
    //                 onChange={handleChange}
    //                 units={"m"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Gaussian Noise:"
    //                 type="number"
    //                 name="gaussianNoise"
    //                 value={camera.gaussianNoise}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Update Rate:"
    //                 type="number"
    //                 name="updateRate"
    //                 value={camera.updateRate}
    //                 onChange={handleChange}
    //                 units={"Hz"}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Image Topic Name:"
    //                 type="text"
    //                 name="imageTopicName"
    //                 style={{ width: "200px" }}
    //                 value={camera.imageTopicName}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Camera Info Topic Name:"
    //                 type="text"
    //                 name="cameraInfoTopicName"
    //                 style={{ width: "200px" }}
    //                 value={camera.cameraInfoTopicName}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //     </>
    // );
}

export default CameraParameters;
