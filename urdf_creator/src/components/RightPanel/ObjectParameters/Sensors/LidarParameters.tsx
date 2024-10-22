import type React from "react";
import type Frame from "../../../../Models/Frame";
import { Lidar } from "../../../../Models/SensorsClass";
import type ThreeScene from "../../../ThreeDisplay/ThreeScene";
import ParameterProps from "../ParameterProps";
import Parameter from "../Parameters/Parameter";
import Property from "../Parameters/Property";

function LidarParameters({
    selectedObject,
    threeScene,
}: {
    threeScene: ThreeScene;
    selectedObject: Frame;
}) {
    if (!selectedObject) return;
    const lidar = selectedObject.sensor;
    if (!(lidar instanceof Lidar)) return;

    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        const { name, value } = e.target;
        const newValue = Number.parseFloat(value);
        switch (name) {
            case "updateRate":
                lidar.updateRate = newValue;
                break;
            case "samples":
                lidar.samples = newValue;
                break;
            case "resolution":
                lidar.resolution = newValue;
                break;
            case "minAngle":
                lidar.minAngle = newValue;
                break;
            case "maxAngle":
                lidar.maxAngle = newValue;
                break;
            case "minRange":
                lidar.minRange = newValue;
                break;
            case "maxRange":
                lidar.maxRange = newValue;
                break;
            case "rangeResolution":
                lidar.rangeResolution = newValue;
                break;
            case "mean":
                lidar.mean = newValue;
                break;
            case "stddev":
                lidar.stddev = newValue;
                break;
        }
        threeScene.forceUpdateCode();
    };
    return null;
    // return (
    //     <>
    //         <Property>
    //             <Parameter
    //                 title="Update Rate:"
    //                 type="text"
    //                 name="updateRate"
    //                 value={lidar.updateRate}
    //                 units="Hz"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Samples:"
    //                 type="text"
    //                 name="samples"
    //                 value={lidar.samples}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Resolution:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="resolution"
    //                 value={lidar.resolution}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Min Angle:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="minAngle"
    //                 value={lidar.minAngle}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Max Angle:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="maxAngle"
    //                 value={lidar.maxAngle}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Min Range:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="minRange"
    //                 value={lidar.minRange}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Max Range:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="maxRange"
    //                 value={lidar.maxRange}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Range Resolution:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="rangeResolution"
    //                 value={lidar.rangeResolution}
    //                 units="&deg; degrees"
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Gaussian Noise Mean:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="mean"
    //                 value={lidar.mean}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //         <Property>
    //             <Parameter
    //                 title="Gaussian Noise Stddev:"
    //                 type="text"
    //                 step="0.000001"
    //                 name="stddev"
    //                 value={lidar.stddev}
    //                 onChange={handleChange}
    //             />
    //         </Property>
    //     </>
    // );
}

export default LidarParameters;
